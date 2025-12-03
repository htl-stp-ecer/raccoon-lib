#include "calibration/motion/analysis/trajectory_analyzer.hpp"
#include <algorithm>
#include <cmath>

namespace libstp::calibration::motion
{
    TrajectoryMetrics analyzeTrajectory(
        const std::vector<TrajectorySnapshot>& trajectory,
        double target)
    {
        TrajectoryMetrics metrics;

        if (trajectory.empty())
            return metrics;

        // Find peak overshoot
        double max_error = 0.0;
        double signed_max_error = 0.0;

        for (const auto& snap : trajectory) {
            double abs_error = std::abs(snap.error);
            if (abs_error > max_error) {
                max_error = abs_error;
                signed_max_error = snap.error;
            }
        }

        metrics.overshoot = (std::abs(target) > 1e-6 && signed_max_error < 0)
            ? (-signed_max_error / std::abs(target))
            : 0.0;

        // Find settling time
        double tolerance = std::max(0.05 * std::abs(target), 0.01);
        const double settle_duration = 0.3;

        for (size_t i = 0; i < trajectory.size(); ++i) {
            if (std::abs(trajectory[i].error) <= tolerance) {
                bool settled = true;
                for (size_t j = i; j < trajectory.size(); ++j) {
                    if (trajectory[j].time - trajectory[i].time > settle_duration)
                        break;
                    if (std::abs(trajectory[j].error) > tolerance) {
                        settled = false;
                        break;
                    }
                }
                if (settled) {
                    metrics.settling_time = trajectory[i].time;
                    break;
                }
            }
        }

        // Calculate steady-state error
        double steady_start_time = trajectory.back().time - 0.5;
        double error_sum = 0.0;
        int count = 0;
        for (const auto& snap : trajectory) {
            if (snap.time >= steady_start_time) {
                error_sum += std::abs(snap.error);
                count++;
            }
        }
        metrics.steady_state_error = (count > 0) ? error_sum / count : 0.0;

        // Calculate ITAE and IAE
        for (size_t i = 1; i < trajectory.size(); ++i) {
            double dt = trajectory[i].time - trajectory[i-1].time;
            double avg_error = (std::abs(trajectory[i].error) + std::abs(trajectory[i-1].error)) / 2.0;
            double avg_time = (trajectory[i].time + trajectory[i-1].time) / 2.0;

            metrics.iae += avg_error * dt;
            metrics.itae += avg_time * avg_error * dt;
        }

        // Count oscillations
        for (size_t i = 1; i < trajectory.size(); ++i) {
            if ((trajectory[i-1].error > 0 && trajectory[i].error < 0) ||
                (trajectory[i-1].error < 0 && trajectory[i].error > 0)) {
                metrics.oscillation_count++;
            }
        }

        // Calculate jerk
        if (trajectory.size() >= 3) {
            std::vector<double> error_accel;
            for (size_t i = 2; i < trajectory.size(); ++i) {
                double dt1 = trajectory[i-1].time - trajectory[i-2].time;
                double dt2 = trajectory[i].time - trajectory[i-1].time;

                if (dt1 > 1e-6 && dt2 > 1e-6) {
                    double v1 = (trajectory[i-1].error - trajectory[i-2].error) / dt1;
                    double v2 = (trajectory[i].error - trajectory[i-1].error) / dt2;
                    double accel = (v2 - v1) / ((dt1 + dt2) / 2.0);
                    error_accel.push_back(accel);
                }
            }

            for (size_t i = 1; i < error_accel.size(); ++i) {
                double dt = (trajectory[i+2].time - trajectory[i+1].time);
                if (dt > 1e-6) {
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

    void analyzeLegacyStepResponse(
        const std::vector<TrajectorySnapshot>& trajectory,
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

    double computeMotionCost(
        const TrajectoryMetrics& metrics,
        const MotionCalibrationConfig& config,
        double max_drift)
    {
        double settling_cost = (metrics.settling_time > 0.01)
            ? metrics.settling_time
            : 20.0;

        double cost = settling_cost;

        if (metrics.overshoot > 0.02) {
            cost += 5000.0 * metrics.overshoot * metrics.overshoot;
        }

        cost += 100.0 * metrics.oscillation_count;
        cost += 0.5 * metrics.itae;
        cost += 0.001 * metrics.total_jerk;

        if (metrics.settling_time > config.target_settling_time * 1.5) {
            cost += 100.0 * (metrics.settling_time - config.target_settling_time);
        }

        cost += 100.0 * metrics.steady_state_error;

        if (max_drift > 0.01) {
            cost += 500.0 * max_drift;
        }

        return cost;
    }
}
