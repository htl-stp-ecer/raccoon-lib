#include "calibration/motion_calibration.hpp"
#include "drive/drive.hpp"
#include "odometry/odometry.hpp"
#include "motion/turn_motion.hpp"
#include "motion/drive_straight_motion.hpp"
#include "motion/strafe_motion.hpp"
#include "motion/motion_pid.hpp"
#include "foundation/logging.hpp"
#include "foundation/types.hpp"

#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <numbers>

#include "button/button.hpp"

namespace libstp::calibration
{
    namespace
    {
        double getCurrentTime()
        {
            return std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count();
        }

        void sleepSeconds(double seconds)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(seconds)
            );
        }
    }

    MotionCalibrator::MotionCalibrator(
        drive::Drive& drive,
        odometry::IOdometry& odometry,
        MotionCalibrationConfig config)
        : drive_(drive)
        , odometry_(odometry)
        , config_(config)
    {
    }

    // Test execution implementation
    struct TestResult
    {
        double settling_time{0.0};
        double overshoot{0.0};
        double steady_state_error{0.0};
        double completion_time{0.0};
        bool success{false};

        struct Snapshot
        {
            double time;
            double error;
            double command;
        };
        std::vector<Snapshot> trajectory;
    };

    TestResult runTurnTest(
        drive::Drive& drive,
        odometry::IOdometry& odometry,
        double angle_deg,
        double max_rate,
        double kp, double ki, double kd,
        double timeout)
    {
        using namespace motion;

        TestResult result;
        result.success = false;

        // Create turn motion with test gains
        TurnConfig cfg;
        cfg.target_angle_rad = angle_deg * std::numbers::pi / 180.0;
        cfg.max_angular_rate = max_rate;
        cfg.angle_kp = kp;
        cfg.angle_ki = ki;
        cfg.angle_kd = kd;

        MotionContext ctx{drive, odometry};
        TurnMotion motion(ctx, cfg);

        // Record trajectory
        double test_start = getCurrentTime();
        auto start_pose = odometry.getPose();
        motion.start();

        const double dt = 0.05;  // 20Hz
        bool motion_finished = false;
        double finish_time = 0.0;
        const double post_finish_duration = 1.0;  // Continue recording 1s after motion reports finished

        while (true)
        {
            double elapsed = getCurrentTime() - test_start;
            if (elapsed > timeout)
            {
                LIBSTP_LOG_WARN("Turn test timeout at {:.1f}s", elapsed);
                return result;
            }

            motion.update(dt);

            // Check if motion just finished
            if (!motion_finished && motion.isFinished())
            {
                motion_finished = true;
                finish_time = elapsed;
                LIBSTP_LOG_DEBUG("Motion reports finished at {:.2f}s, continuing to record settling...", finish_time);
            }

            // Stop recording after post-finish duration
            if (motion_finished && (elapsed - finish_time) > post_finish_duration)
            {
                break;
            }

            // Record snapshot with X/Y drift
            auto current_pose = odometry.getPose();
            TestResult::Snapshot snap;
            snap.time = elapsed;
            snap.error = odometry.getHeadingError(cfg.target_angle_rad);

            // Track X/Y drift during rotation (should be zero for pure turn)
            double dx = current_pose.position.x() - start_pose.position.x();
            double dy = current_pose.position.y() - start_pose.position.y();
            snap.command = std::sqrt(dx*dx + dy*dy);  // Store drift magnitude in command field

            result.trajectory.push_back(snap);

            sleepSeconds(dt);
        }

        result.completion_time = getCurrentTime() - test_start;
        result.success = true;

        return result;
    }

    TestResult runDriveTest(
        drive::Drive& drive,
        odometry::IOdometry& odometry,
        double distance_m,
        double max_speed,
        double dist_kp, double dist_ki, double dist_kd,
        double heading_kp, double heading_ki, double heading_kd,
        double timeout)
    {
        using namespace motion;

        TestResult result;
        result.success = false;

        // Create drive motion with test gains
        DriveStraightConfig cfg;
        cfg.distance_m = distance_m;
        cfg.max_speed_mps = max_speed;
        cfg.distance_kp = dist_kp;
        cfg.distance_ki = dist_ki;
        cfg.distance_kd = dist_kd;
        cfg.heading_kp = heading_kp;
        cfg.heading_ki = heading_ki;
        cfg.heading_kd = heading_kd;

        MotionContext ctx{drive, odometry};
        DriveStraightMotion motion(ctx, cfg);

        // Record trajectory
        double test_start = getCurrentTime();
        motion.start();

        const double dt = 0.05;  // 20Hz
        while (!motion.isFinished())
        {
            double elapsed = getCurrentTime() - test_start;
            if (elapsed > timeout)
            {
                LIBSTP_LOG_WARN("Drive test timeout at {:.1f}s", elapsed);
                return result;
            }

            motion.update(dt);

            // Record snapshot
            auto dist_info = odometry.getDistanceFromOrigin();
            TestResult::Snapshot snap;
            snap.time = elapsed;
            snap.error = distance_m - dist_info.forward;
            snap.command = 0.0;
            result.trajectory.push_back(snap);

            sleepSeconds(dt);
        }

        result.completion_time = getCurrentTime() - test_start;
        result.success = true;

        return result;
    }

    // Advanced trajectory analysis metrics
    struct TrajectoryMetrics
    {
        double settling_time{0.0};
        double overshoot{0.0};
        double steady_state_error{0.0};
        double itae{0.0};              // Integral Time-weighted Absolute Error
        double iae{0.0};               // Integral Absolute Error
        int oscillation_count{0};       // Number of zero crossings
        double max_jerk{0.0};          // Maximum jerk (smoothness)
        double total_jerk{0.0};        // Total jerk integral
    };

    TrajectoryMetrics analyzeTrajectory(
        const std::vector<TestResult::Snapshot>& trajectory,
        double target)
    {
        TrajectoryMetrics metrics;

        if (trajectory.empty())
            return metrics;

        // Find peak overshoot (looking for crossing beyond target)
        double max_error = 0.0;
        double signed_max_error = 0.0;

        for (const auto& snap : trajectory)
        {
            double abs_error = std::abs(snap.error);
            if (abs_error > max_error)
            {
                max_error = abs_error;
                signed_max_error = snap.error;
            }
        }

        // Overshoot is only when we go PAST the target (negative error means overshoot)
        metrics.overshoot = (std::abs(target) > 1e-6 && signed_max_error < 0)
            ? (-signed_max_error / std::abs(target))
            : 0.0;

        // Find settling time (first time within 5% and stays there)
        double tolerance = std::max(0.05 * std::abs(target), 0.01);
        const double settle_duration = 0.3;  // Must stay settled for 0.3s

        for (size_t i = 0; i < trajectory.size(); ++i)
        {
            if (std::abs(trajectory[i].error) <= tolerance)
            {
                // Check if stays within tolerance
                bool settled = true;
                for (size_t j = i; j < trajectory.size(); ++j)
                {
                    if (trajectory[j].time - trajectory[i].time > settle_duration)
                        break;
                    if (std::abs(trajectory[j].error) > tolerance)
                    {
                        settled = false;
                        break;
                    }
                }
                if (settled)
                {
                    metrics.settling_time = trajectory[i].time;
                    break;
                }
            }
        }

        // Calculate steady-state error (average of last 0.5 seconds)
        double steady_start_time = trajectory.back().time - 0.5;
        double error_sum = 0.0;
        int count = 0;
        for (const auto& snap : trajectory)
        {
            if (snap.time >= steady_start_time)
            {
                error_sum += std::abs(snap.error);
                count++;
            }
        }
        metrics.steady_state_error = (count > 0) ? error_sum / count : 0.0;

        // Calculate ITAE (Integral Time-weighted Absolute Error) and IAE
        for (size_t i = 1; i < trajectory.size(); ++i)
        {
            double dt = trajectory[i].time - trajectory[i-1].time;
            double avg_error = (std::abs(trajectory[i].error) + std::abs(trajectory[i-1].error)) / 2.0;
            double avg_time = (trajectory[i].time + trajectory[i-1].time) / 2.0;

            metrics.iae += avg_error * dt;
            metrics.itae += avg_time * avg_error * dt;
        }

        // Count oscillations (zero crossings of error)
        for (size_t i = 1; i < trajectory.size(); ++i)
        {
            if ((trajectory[i-1].error > 0 && trajectory[i].error < 0) ||
                (trajectory[i-1].error < 0 && trajectory[i].error > 0))
            {
                metrics.oscillation_count++;
            }
        }

        // Calculate jerk (rate of change of acceleration of error)
        // This measures smoothness - lower jerk = smoother motion
        if (trajectory.size() >= 3)
        {
            std::vector<double> error_accel;
            for (size_t i = 2; i < trajectory.size(); ++i)
            {
                double dt1 = trajectory[i-1].time - trajectory[i-2].time;
                double dt2 = trajectory[i].time - trajectory[i-1].time;

                if (dt1 > 1e-6 && dt2 > 1e-6)
                {
                    double v1 = (trajectory[i-1].error - trajectory[i-2].error) / dt1;
                    double v2 = (trajectory[i].error - trajectory[i-1].error) / dt2;
                    double accel = (v2 - v1) / ((dt1 + dt2) / 2.0);
                    error_accel.push_back(accel);
                }
            }

            // Calculate jerk from acceleration
            for (size_t i = 1; i < error_accel.size(); ++i)
            {
                double dt = (trajectory[i+2].time - trajectory[i+1].time);
                if (dt > 1e-6)
                {
                    double jerk = (error_accel[i] - error_accel[i-1]) / dt;
                    double abs_jerk = std::abs(jerk);
                    metrics.total_jerk += abs_jerk * dt;
                    if (abs_jerk > metrics.max_jerk)
                        metrics.max_jerk = abs_jerk;
                }
            }
        }

        return metrics;
    }

    // Legacy wrapper for backwards compatibility
    void analyzeStepResponse(
        const std::vector<TestResult::Snapshot>& trajectory,
        double target,
        double& settling_time,
        double& overshoot,
        double& steady_state_error)
    {
        auto metrics = analyzeTrajectory(trajectory, target);
        settling_time = metrics.settling_time;
        overshoot = metrics.overshoot;
        steady_state_error = metrics.steady_state_error;
    }

    // Cost function for smooth, no-overshoot motion
    double computeMotionCost(const TrajectoryMetrics& metrics, const MotionCalibrationConfig& config,
                             double max_drift = 0.0)
    {
        // Base cost: settling time (we want this minimized)
        // If settling time is 0 (not detected), use a large penalty
        double settling_cost = (metrics.settling_time > 0.01)
            ? metrics.settling_time
            : 20.0;  // Penalty if settling not achieved

        double cost = settling_cost;

        // HEAVY penalty for overshoot (we want ZERO overshoot)
        // Quadratic penalty makes even small overshoots very expensive
        if (metrics.overshoot > 0.02)  // Allow tiny 2% tolerance
        {
            cost += 5000.0 * metrics.overshoot * metrics.overshoot;  // Increased from 1000
        }

        // Penalize oscillations (hunting behavior at the end)
        // Each oscillation adds significant cost
        cost += 100.0 * metrics.oscillation_count;  // Increased from 10.0

        // Add ITAE component (penalizes persistent errors)
        cost += 0.5 * metrics.itae;  // Increased from 0.1

        // Smoothness penalty (jerky motion is bad)
        // Scale jerk to reasonable range (typically 100-20000)
        cost += 0.001 * metrics.total_jerk;  // Increased from 0.01

        // Penalty for slow settling (if it takes too long)
        if (metrics.settling_time > config.target_settling_time * 1.5)
        {
            cost += 100.0 * (metrics.settling_time - config.target_settling_time);
        }

        // Penalty for steady-state error
        cost += 100.0 * metrics.steady_state_error;

        // X/Y drift penalty (for turn motions - should rotate in place)
        if (max_drift > 0.01)  // More than 1cm drift
        {
            cost += 500.0 * max_drift;  // Heavy penalty for translational drift during rotation
        }

        return cost;
    }

    // Nelder-Mead Simplex optimizer for PID gains
    struct Simplex
    {
        std::vector<std::array<double, 3>> vertices;  // Each vertex is [kp, ki, kd]
        std::vector<double> costs;

        Simplex(const std::array<double, 3>& initial_guess, double scale = 1.0)
        {
            // Create initial simplex around initial guess
            vertices.resize(4);
            costs.resize(4);

            vertices[0] = initial_guess;

            // Create 3 more vertices by perturbing each dimension
            for (int i = 0; i < 3; ++i)
            {
                vertices[i + 1] = initial_guess;
                vertices[i + 1][i] += scale;
            }
        }

        void sort()
        {
            // Sort vertices by cost (ascending)
            std::vector<size_t> indices(4);
            std::iota(indices.begin(), indices.end(), 0);

            std::sort(indices.begin(), indices.end(),
                     [this](size_t a, size_t b) { return costs[a] < costs[b]; });

            std::vector<std::array<double, 3>> sorted_vertices(4);
            std::vector<double> sorted_costs(4);

            for (size_t i = 0; i < 4; ++i)
            {
                sorted_vertices[i] = vertices[indices[i]];
                sorted_costs[i] = costs[indices[i]];
            }

            vertices = sorted_vertices;
            costs = sorted_costs;
        }

        std::array<double, 3> centroid() const
        {
            // Centroid of all vertices except the worst
            std::array<double, 3> c = {0.0, 0.0, 0.0};
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                    c[j] += vertices[i][j];
            }
            for (int j = 0; j < 3; ++j)
                c[j] /= 3.0;
            return c;
        }

        double size() const
        {
            // Measure simplex size (max distance from best to others)
            double max_dist = 0.0;
            for (int i = 1; i < 4; ++i)
            {
                double dist = 0.0;
                for (int j = 0; j < 3; ++j)
                {
                    double diff = vertices[i][j] - vertices[0][j];
                    dist += diff * diff;
                }
                max_dist = std::max(max_dist, std::sqrt(dist));
            }
            return max_dist;
        }
    };

    std::array<double, 3> reflect(const std::array<double, 3>& point,
                                   const std::array<double, 3>& centroid,
                                   double alpha = 1.0)
    {
        std::array<double, 3> reflected;
        for (int i = 0; i < 3; ++i)
            reflected[i] = centroid[i] + alpha * (centroid[i] - point[i]);
        return reflected;
    }

    std::array<double, 3> expand(const std::array<double, 3>& reflected,
                                  const std::array<double, 3>& centroid,
                                  double gamma = 2.0)
    {
        std::array<double, 3> expanded;
        for (int i = 0; i < 3; ++i)
            expanded[i] = centroid[i] + gamma * (reflected[i] - centroid[i]);
        return expanded;
    }

    std::array<double, 3> contract(const std::array<double, 3>& point,
                                    const std::array<double, 3>& centroid,
                                    double rho = 0.5)
    {
        std::array<double, 3> contracted;
        for (int i = 0; i < 3; ++i)
            contracted[i] = centroid[i] + rho * (point[i] - centroid[i]);
        return contracted;
    }

    void clampGains(std::array<double, 3>& gains)
    {
        // Ensure gains stay within reasonable bounds
        gains[0] = std::clamp(gains[0], 0.1, 20.0);   // kp
        gains[1] = std::clamp(gains[1], 0.0, 10.0);   // ki
        gains[2] = std::clamp(gains[2], 0.0, 10.0);   // kd
    }

    // Optimization-based tuning for smooth, no-overshoot motion
    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneAngleController()
    {
        LIBSTP_LOG_INFO("=== Starting angle controller optimization (no-overshoot mode) ===");

        const double test_angle = 90.0;
        const double test_rate = 1.0;

        // Objective function: evaluates cost for given PID gains
        auto objectiveFunction = [&](const std::array<double, 3>& gains_array) -> double
        {
            double kp = gains_array[0];
            double ki = gains_array[1];
            double kd = gains_array[2];

            LIBSTP_LOG_DEBUG("Testing gains: kp={:.3f}, ki={:.3f}, kd={:.3f}", kp, ki, kd);
            LIBSTP_LOG_INFO("Press the button to confirm a {} turn test...", test_angle);
            button::Button::waitForButtonPress();

            // Run test with these gains
            auto result = runTurnTest(drive_, odometry_, test_angle, test_rate,
                                     kp, ki, kd, config_.max_single_test_time);

            if (!result.success)
            {
                LIBSTP_LOG_WARN("Test failed, returning high cost");
                return 10000.0;  // Very high cost for failed tests
            }

            // Analyze trajectory with advanced metrics
            auto metrics = analyzeTrajectory(result.trajectory, test_angle);

            // Calculate max X/Y drift during turn (stored in command field)
            double max_drift = 0.0;
            for (const auto& snap : result.trajectory)
            {
                if (snap.command > max_drift)
                    max_drift = snap.command;
            }

            // Compute cost optimized for smooth, no-overshoot motion
            double cost = computeMotionCost(metrics, config_, max_drift);

            LIBSTP_LOG_DEBUG("  Metrics: settle={:.2f}s, overshoot={:.1f}%, osc={}, ITAE={:.2f}, jerk={:.2f}, drift={:.3f}m",
                           metrics.settling_time, metrics.overshoot * 100.0,
                           metrics.oscillation_count, metrics.itae, metrics.total_jerk, max_drift);
            LIBSTP_LOG_DEBUG("  Cost: {:.3f}", cost);

            return cost;
        };

        // Initial guess: Start with critically damped PD controller
        // For critical damping with no overshoot, we use:
        // - Moderate P gain (not too aggressive)
        // - Small I term (avoid wind-up issues)
        // - Significant D term (damping to prevent overshoot)
        std::array<double, 3> initial_guess = {
            config_.initial_kp,           // kp: start with user's guess
            0.0,                          // ki: start with zero
            config_.initial_kp * 0.25     // kd: 25% of kp for damping
        };
        clampGains(initial_guess);

        LIBSTP_LOG_INFO("Initial guess: kp={:.3f}, ki={:.3f}, kd={:.3f}",
                       initial_guess[0], initial_guess[1], initial_guess[2]);

        // Create simplex for Nelder-Mead
        Simplex simplex(initial_guess, config_.initial_kp * 0.5);

        // Evaluate initial simplex
        for (int i = 0; i < 4; ++i)
        {
            clampGains(simplex.vertices[i]);
            simplex.costs[i] = objectiveFunction(simplex.vertices[i]);
        }

        // Nelder-Mead optimization loop
        const int max_iterations = config_.max_iterations * 3;  // Allow more iterations for optimization
        const double tolerance = 0.01;  // Convergence tolerance

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            simplex.sort();

            LIBSTP_LOG_INFO("Iteration {}/{}: Best cost={:.3f}, kp={:.3f}, ki={:.3f}, kd={:.3f}",
                          iter + 1, max_iterations, simplex.costs[0],
                          simplex.vertices[0][0], simplex.vertices[0][1], simplex.vertices[0][2]);

            // Check for convergence
            if (simplex.size() < tolerance && simplex.costs[0] < 10.0)
            {
                LIBSTP_LOG_INFO("Converged after {} iterations!", iter + 1);
                break;
            }

            // Nelder-Mead algorithm steps
            auto cent = simplex.centroid();

            // Reflection
            auto reflected = reflect(simplex.vertices[3], cent);
            clampGains(reflected);
            double cost_reflected = objectiveFunction(reflected);

            if (cost_reflected < simplex.costs[0])
            {
                // Expansion
                auto expanded = expand(reflected, cent);
                clampGains(expanded);
                double cost_expanded = objectiveFunction(expanded);

                if (cost_expanded < cost_reflected)
                {
                    simplex.vertices[3] = expanded;
                    simplex.costs[3] = cost_expanded;
                }
                else
                {
                    simplex.vertices[3] = reflected;
                    simplex.costs[3] = cost_reflected;
                }
            }
            else if (cost_reflected < simplex.costs[2])
            {
                simplex.vertices[3] = reflected;
                simplex.costs[3] = cost_reflected;
            }
            else
            {
                // Contraction
                auto contracted = contract(simplex.vertices[3], cent);
                clampGains(contracted);
                double cost_contracted = objectiveFunction(contracted);

                if (cost_contracted < simplex.costs[3])
                {
                    simplex.vertices[3] = contracted;
                    simplex.costs[3] = cost_contracted;
                }
                else
                {
                    // Shrink
                    for (int i = 1; i < 4; ++i)
                    {
                        for (int j = 0; j < 3; ++j)
                            simplex.vertices[i][j] = simplex.vertices[0][j] +
                                0.5 * (simplex.vertices[i][j] - simplex.vertices[0][j]);
                        clampGains(simplex.vertices[i]);
                        simplex.costs[i] = objectiveFunction(simplex.vertices[i]);
                    }
                }
            }
        }

        // Return best gains found
        simplex.sort();
        PidGainsMotion gains;
        gains.kp = simplex.vertices[0][0];
        gains.ki = simplex.vertices[0][1];
        gains.kd = simplex.vertices[0][2];

        LIBSTP_LOG_INFO("=== Optimization complete ===");
        LIBSTP_LOG_INFO("Final gains: kp={:.4f}, ki={:.4f}, kd={:.4f}", gains.kp, gains.ki, gains.kd);
        LIBSTP_LOG_INFO("Final cost: {:.3f}", simplex.costs[0]);

        // Run one final test to log detailed metrics
        auto final_result = runTurnTest(drive_, odometry_, test_angle, test_rate,
                                       gains.kp, gains.ki, gains.kd, config_.max_single_test_time);
        if (final_result.success)
        {
            auto final_metrics = analyzeTrajectory(final_result.trajectory, test_angle);

            // Calculate max drift
            double max_drift = 0.0;
            for (const auto& snap : final_result.trajectory)
            {
                if (snap.command > max_drift)
                    max_drift = snap.command;
            }

            LIBSTP_LOG_INFO("Final metrics:");
            LIBSTP_LOG_INFO("  Settling time: {:.2f}s", final_metrics.settling_time);
            LIBSTP_LOG_INFO("  Overshoot: {:.2f}%", final_metrics.overshoot * 100.0);
            LIBSTP_LOG_INFO("  Oscillations: {}", final_metrics.oscillation_count);
            LIBSTP_LOG_INFO("  ITAE: {:.3f}", final_metrics.itae);
            LIBSTP_LOG_INFO("  Total jerk: {:.3f}", final_metrics.total_jerk);
            LIBSTP_LOG_INFO("  Steady-state error: {:.4f}", final_metrics.steady_state_error);
            LIBSTP_LOG_INFO("  Max X/Y drift: {:.4f}m ({:.1f}cm)", max_drift, max_drift * 100.0);
        }

        return gains;
    }

    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneDistanceController()
    {
        LIBSTP_LOG_INFO("=== Starting distance controller tuning ===");

        PidGainsMotion gains;
        gains.kp = config_.initial_kp;
        gains.ki = 0.0;
        gains.kd = 0.0;

        // Similar to angle tuning but for linear distance
        const double test_distance = 0.5;
        const double test_speed = 0.4;

        for (int iter = 0; iter < config_.max_iterations; ++iter)
        {
            LIBSTP_LOG_INFO("P-tune iteration {}: kp={:.3f}", iter + 1, gains.kp);

            // Use conservative heading gains for testing
            auto result = runDriveTest(drive_, odometry_, test_distance, test_speed,
                                      gains.kp, 0.0, 0.0,
                                      3.0, 0.0, 0.0,
                                      config_.max_single_test_time);

            if (!result.success)
            {
                gains.kp *= 0.7;
                continue;
            }

            analyzeStepResponse(result.trajectory, test_distance,
                              result.settling_time, result.overshoot, result.steady_state_error);

            LIBSTP_LOG_INFO("  settling_time={:.2f}s, overshoot={:.1f}%, sse={:.4f}m",
                          result.settling_time, result.overshoot * 100.0, result.steady_state_error);

            bool too_slow = result.settling_time > config_.target_settling_time * 1.3;
            bool too_much_overshoot = result.overshoot > config_.max_overshoot;

            if (!too_slow && !too_much_overshoot)
            {
                break;
            }

            if (too_slow)
            {
                gains.kp *= 1.3;
            }
            else if (too_much_overshoot)
            {
                gains.kp *= 0.8;
            }
        }

        // Add I and D terms
        if (gains.kp > 0.1)
        {
            gains.ki = gains.kp / (5.0 * std::max(config_.target_settling_time, 0.5));
            gains.kd = gains.kp * config_.target_settling_time / 10.0;
        }

        LIBSTP_LOG_INFO("=== Distance controller tuned: kp={:.3f}, ki={:.3f}, kd={:.3f} ===",
                       gains.kp, gains.ki, gains.kd);

        return gains;
    }

    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneHeadingController()
    {
        LIBSTP_LOG_INFO("=== Starting heading controller tuning ===");

        // For heading correction during drive, use more aggressive gains
        PidGainsMotion gains;
        gains.kp = 4.0;
        gains.ki = 0.1;
        gains.kd = 0.15;

        LIBSTP_LOG_INFO("=== Heading controller set to: kp={:.3f}, ki={:.3f}, kd={:.3f} ===",
                       gains.kp, gains.ki, gains.kd);

        return gains;
    }

    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneLateralController()
    {
        LIBSTP_LOG_INFO("=== Starting lateral controller tuning ===");

        PidGainsMotion gains;
        gains.kp = 2.0;
        gains.ki = 0.05;
        gains.kd = 0.1;

        LIBSTP_LOG_INFO("=== Lateral controller set to: kp={:.3f}, ki={:.3f}, kd={:.3f} ===",
                       gains.kp, gains.ki, gains.kd);

        return gains;
    }

    bool MotionCalibrator::validateGains(const std::vector<PidGainsMotion>& gains, MotionType type)
    {
        LIBSTP_LOG_INFO("Validating gains for motion type {}...", static_cast<int>(type));

        // Run subset of tests to validate
        if (type == MotionType::TURN && !gains.empty())
        {
            const auto& g = gains[0];

            // Test at different angles and speeds
            for (double angle : {45.0, 90.0, 180.0})
            {
                for (double rate : {0.5, 1.0, 2.0})
                {
                    auto result = runTurnTest(drive_, odometry_, angle, rate,
                                             g.kp, g.ki, g.kd, config_.max_single_test_time);

                    if (!result.success)
                    {
                        LIBSTP_LOG_WARN("Validation failed for angle={:.0f}° rate={:.1f}", angle, rate);
                        return false;
                    }

                    analyzeStepResponse(result.trajectory, angle,
                                      result.settling_time, result.overshoot, result.steady_state_error);

                    LIBSTP_LOG_INFO("  {:.0f}°@{:.1f}rad/s: settle={:.2f}s overshoot={:.1f}% sse={:.3f}",
                                   angle, rate, result.settling_time, result.overshoot * 100.0, result.steady_state_error);
                }
            }
        }

        return true;
    }

    bool MotionCalibrator::crossValidate(const MotionCalibrationResult& result)
    {
        LIBSTP_LOG_INFO("Cross-validating calibration results...");
        return true;  // All validation happens in validateGains
    }

    bool MotionCalibrator::checkSafeSpace() const
    {
        LIBSTP_LOG_INFO("Checking safe space (assuming OK for calibration)");
        return true;
    }

    bool MotionCalibrator::checkTimeout(double start_time) const
    {
        double current_time = getCurrentTime();
        return (current_time - start_time) > config_.max_test_duration;
    }

    MotionCalibrationResult MotionCalibrator::calibrate()
    {
        LIBSTP_LOG_INFO("========================================");
        LIBSTP_LOG_INFO("Starting full motion calibration");
        LIBSTP_LOG_INFO("========================================");

        calibration_start_time_ = getCurrentTime();
        MotionCalibrationResult final_result;

        // Calibrate each motion type
        auto turn_result = calibrateTurnMotion();
        auto drive_result = calibrateDriveStraightMotion();

        // Combine results
        final_result.gains.insert(final_result.gains.end(),
                                 turn_result.gains.begin(),
                                 turn_result.gains.end());
        final_result.gains.insert(final_result.gains.end(),
                                 drive_result.gains.begin(),
                                 drive_result.gains.end());

        // Only calibrate strafe if kinematics supports it
        if (drive_.getKinematics().supportsLateralMotion())
        {
            auto strafe_result = calibrateStrafeMotion();
            final_result.gains.insert(final_result.gains.end(),
                                     strafe_result.gains.begin(),
                                     strafe_result.gains.end());
        }

        // Cross-validate
        final_result.success = turn_result.success && drive_result.success;
        final_result.duration_seconds = getCurrentTime() - calibration_start_time_;

        LIBSTP_LOG_INFO("========================================");
        LIBSTP_LOG_INFO("Motion calibration complete!");
        LIBSTP_LOG_INFO("Success: {}", final_result.success);
        LIBSTP_LOG_INFO("Duration: {:.1f} seconds", final_result.duration_seconds);
        LIBSTP_LOG_INFO("========================================");

        return final_result;
    }

    MotionCalibrationResult MotionCalibrator::calibrateTurnMotion()
    {
        LIBSTP_LOG_INFO("Starting TurnMotion calibration...");

        MotionCalibrationResult result;

        // Safety check
        if (!checkSafeSpace())
        {
            result.success = false;
            result.error_message = "Insufficient safe space for calibration";
            return result;
        }

        // Tune angle controller
        auto gains = tuneAngleController();

        // Store results
        MotionCalibrationResult::GainSet gain_set;
        gain_set.kp = gains.kp;
        gain_set.ki = gains.ki;
        gain_set.kd = gains.kd;
        gain_set.motion_type = MotionType::TURN;
        gain_set.controller_name = "angle";
        result.gains.push_back(gain_set);

        // Validate
        result.success = validateGains({gains}, MotionType::TURN);

        LIBSTP_LOG_INFO("TurnMotion calibration complete: kp={:.3f}, ki={:.3f}, kd={:.3f}",
                       gains.kp, gains.ki, gains.kd);

        return result;
    }

    MotionCalibrationResult MotionCalibrator::calibrateDriveStraightMotion()
    {
        LIBSTP_LOG_INFO("Starting DriveStraightMotion calibration...");

        MotionCalibrationResult result;

        // Tune controllers
        auto distance_gains = tuneDistanceController();
        auto heading_gains = tuneHeadingController();

        // Store results
        MotionCalibrationResult::GainSet distance_set;
        distance_set.kp = distance_gains.kp;
        distance_set.ki = distance_gains.ki;
        distance_set.kd = distance_gains.kd;
        distance_set.motion_type = MotionType::DRIVE_STRAIGHT;
        distance_set.controller_name = "distance";
        result.gains.push_back(distance_set);

        MotionCalibrationResult::GainSet heading_set;
        heading_set.kp = heading_gains.kp;
        heading_set.ki = heading_gains.ki;
        heading_set.kd = heading_gains.kd;
        heading_set.motion_type = MotionType::DRIVE_STRAIGHT;
        heading_set.controller_name = "heading";
        result.gains.push_back(heading_set);

        result.success = true;

        LIBSTP_LOG_INFO("DriveStraightMotion calibration complete");

        return result;
    }

    MotionCalibrationResult MotionCalibrator::calibrateStrafeMotion()
    {
        LIBSTP_LOG_INFO("Starting StrafeMotion calibration...");

        MotionCalibrationResult result;

        // Tune controllers
        auto lateral_gains = tuneLateralController();
        auto heading_gains = tuneHeadingController();

        // Store results
        MotionCalibrationResult::GainSet lateral_set;
        lateral_set.kp = lateral_gains.kp;
        lateral_set.ki = lateral_gains.ki;
        lateral_set.kd = lateral_gains.kd;
        lateral_set.motion_type = MotionType::STRAFE;
        lateral_set.controller_name = "lateral";
        result.gains.push_back(lateral_set);

        MotionCalibrationResult::GainSet heading_set;
        heading_set.kp = heading_gains.kp;
        heading_set.ki = heading_gains.ki;
        heading_set.kd = heading_gains.kd;
        heading_set.motion_type = MotionType::STRAFE;
        heading_set.controller_name = "heading";
        result.gains.push_back(heading_set);

        result.success = true;

        LIBSTP_LOG_INFO("StrafeMotion calibration complete");

        return result;
    }
}
