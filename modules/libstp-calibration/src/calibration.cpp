//
// Created by tobias on 27/11/25.
//


#include "calibration/calibration.hpp"
#include "drive/motor_adapter.hpp"
#include "foundation/logging.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <thread>

using namespace libstp::drive;
using namespace libstp;

namespace {
    // Helper functions
    double getMeanValue(const std::vector<double>& values) {
        if (values.empty()) return 0.0;
        return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    }

    double getStdDev(const std::vector<double>& values, double mean) {
        if (values.size() < 2) return 0.0;
        double sum_sq_diff = 0.0;
        for (double v : values) {
            double diff = v - mean;
            sum_sq_diff += diff * diff;
        }
        return std::sqrt(sum_sq_diff / (values.size() - 1));
    }

    double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
}

MotorCalibrator::MotorCalibrator(hal::motor::Motor& motor, const CalibrationConfig& config)
    : motor_(motor), config_(config)
{
    LIBSTP_LOG_INFO("MotorCalibrator created for motor port {}", motor_.port);

    // Create a motor adapter for this motor
    adapter_ = std::make_unique<MotorAdapter>(&motor_);
}

CalibrationResult MotorCalibrator::calibrate()
{
    LIBSTP_LOG_INFO("=== Starting motor calibration ===");
    calibration_start_time_ = getCurrentTime();
    result_ = CalibrationResult{};
    result_.success = false;
    emergency_stop_ = false;
    total_distance_moved_ = 0.0;

    try {
        // Step 1: Calibrate feedforward
        LIBSTP_LOG_INFO("Step 1/3: Calibrating feedforward parameters...");
        result_.ff = calibrateFeedforward();

        if (checkTimeout(calibration_start_time_)) {
            result_.error_message = "Calibration timeout during feedforward calibration";
            LIBSTP_LOG_ERROR("{}", result_.error_message);
            return result_;
        }

        // Step 2: Calibrate PID using the measured feedforward
        LIBSTP_LOG_INFO("Step 2/3: Calibrating PID parameters...");
        result_.pid = calibratePID(result_.ff);

        if (checkTimeout(calibration_start_time_)) {
            result_.error_message = "Calibration timeout during PID calibration";
            LIBSTP_LOG_ERROR("{}", result_.error_message);
            return result_;
        }

        // Step 3: Validate the calibration
        LIBSTP_LOG_INFO("Step 3/3: Validating calibration...");
        if (!validateCalibration(result_.pid, result_.ff, result_.metrics)) {
            result_.error_message = "Calibration validation failed";
            LIBSTP_LOG_WARN("{}", result_.error_message);
            // Don't return - validation failure isn't fatal
        }

        // Check parameter ranges
        if (!validateParameterRanges(result_.pid, result_.ff)) {
            result_.error_message = "Calibrated parameters out of acceptable range";
            LIBSTP_LOG_ERROR("{}", result_.error_message);
            return result_;
        }

        result_.duration_seconds = getCurrentTime() - calibration_start_time_;
        result_.success = true;

        LIBSTP_LOG_INFO("=== Calibration completed successfully ===");
        LIBSTP_LOG_INFO("PID gains: kp={:.3f}, ki={:.3f}, kd={:.3f}",
                    result_.pid.kp, result_.pid.ki, result_.pid.kd);
        LIBSTP_LOG_INFO("Feedforward: kS={:.3f}, kV={:.3f}, kA={:.3f}",
                    result_.ff.kS, result_.ff.kV, result_.ff.kA);
        LIBSTP_LOG_INFO("Duration: {:.2f} seconds", result_.duration_seconds);

    } catch (const std::exception& e) {
        result_.error_message = std::string("Calibration exception: ") + e.what();
        LIBSTP_LOG_ERROR("{}", result_.error_message);
        stopMotor();
    }

    stopMotor();
    return result_;
}

foundation::Feedforward MotorCalibrator::calibrateFeedforward()
{
    foundation::Feedforward ff;

    // Find static friction threshold (for reference/debugging only)
    LIBSTP_LOG_INFO("Finding static friction threshold (for reference)...");
    double kS_threshold = findStaticFriction();
    LIBSTP_LOG_INFO("Static friction threshold = {:.2f}%", kS_threshold);

    // Find velocity constant (kV) and kS from linear regression
    // This will update result_.metrics with the regression-based kS
    LIBSTP_LOG_INFO("Finding velocity constant via linear regression...");
    double kV_percent = findVelocityConstant();
    // Get kS from the regression (stored in metrics by findVelocityConstant)
    double kS_percent = result_.metrics.static_friction_forward;
    LIBSTP_LOG_INFO("Linear regression results: kS={:.2f}%, kV={:.3f}% per rad/s", kS_percent, kV_percent);

    // Find acceleration constant (kA) - this returns percent per rad/s²
    LIBSTP_LOG_INFO("Finding acceleration constant...");
    double kA_percent = findAccelerationConstant(kS_percent, kV_percent);
    LIBSTP_LOG_INFO("Acceleration constant kA = {:.3f}% per rad/s²", kA_percent);

    // Convert from percent [0-100] to normalized [0-1] for VelocityController
    // VelocityController expects normalized units where 1.0 = 100% motor command
    ff.kS = kS_percent / 100.0;
    ff.kV = kV_percent / 100.0;
    ff.kA = kA_percent / 100.0;

    LIBSTP_LOG_INFO("Converted to normalized units: kS={:.4f}, kV={:.4f}, kA={:.4f}", ff.kS, ff.kV, ff.kA);

    return ff;
}

foundation::PidGains MotorCalibrator::calibratePID(const foundation::Feedforward& ff)
{
    // Apply feedforward to the controller for PID tuning
    adapter_->getController().setFF(ff);

    if (config_.use_relay_feedback) {
        LIBSTP_LOG_INFO("Using relay feedback tuning (aggressive mode)");
        return relayFeedbackTuning(ff);
    } else {
        LIBSTP_LOG_INFO("Using step response tuning (conservative mode)");
        return stepResponseTuning(ff);
    }
}

double MotorCalibrator::findStaticFriction()
{
    const double start_time = getCurrentTime();

    // Test forward direction
    LIBSTP_LOG_DEBUG("Testing forward static friction...");
    resetMotor();

    double forward_threshold = 0.0;
    for (double cmd = 0.0; cmd <= config_.static_friction_max; cmd += config_.static_friction_increment) {
        setMotorCommand(cmd);

        // Wait for motor to respond and update velocity estimate
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        adapter_->updateEncoderVelocity(0.05);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        adapter_->updateEncoderVelocity(0.1);

        double velocity = std::abs(getMotorVelocity());
        LIBSTP_LOG_DEBUG("Forward cmd={:.2f}%, velocity={:.3f} rad/s", cmd, velocity);

        if (velocity > config_.min_velocity_threshold) {
            forward_threshold = cmd;
            LIBSTP_LOG_DEBUG("Forward threshold found at {:.2f}%", cmd);
            break;
        }

        if (checkTimeout(start_time)) break;
    }

    stopMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Test backward direction
    LIBSTP_LOG_DEBUG("Testing backward static friction...");
    resetMotor();

    double backward_threshold = 0.0;
    for (double cmd = 0.0; cmd <= config_.static_friction_max; cmd += config_.static_friction_increment) {
        setMotorCommand(-cmd);

        // Wait for motor to respond and update velocity estimate
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        adapter_->updateEncoderVelocity(0.05);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        adapter_->updateEncoderVelocity(0.1);

        double velocity = std::abs(getMotorVelocity());
        LIBSTP_LOG_DEBUG("Backward cmd={:.2f}%, velocity={:.3f} rad/s", -cmd, velocity);

        if (velocity > config_.min_velocity_threshold) {
            backward_threshold = cmd;
            LIBSTP_LOG_DEBUG("Backward threshold found at {:.2f}%", cmd);
            break;
        }

        if (checkTimeout(start_time)) break;
    }

    stopMotor();

    // Store metrics
    result_.metrics.static_friction_forward = forward_threshold;
    result_.metrics.static_friction_backward = backward_threshold;

    // Return average
    double kS = (forward_threshold + backward_threshold) / 2.0;

    // Validate
    if (kS < config_.ranges.kS_min || kS > config_.ranges.kS_max) {
        LIBSTP_LOG_WARN("Static friction {:.2f} outside expected range [{:.2f}, {:.2f}]",
                    kS, config_.ranges.kS_min, config_.ranges.kS_max);
    }

    return kS;
}

double MotorCalibrator::findVelocityConstant()
{
    const double start_time = getCurrentTime();

    std::vector<double> commands;
    std::vector<double> velocities;

    for (double cmd : config_.velocity_test_commands) {
        if (checkTimeout(start_time)) break;

        LIBSTP_LOG_DEBUG("Testing velocity at {:.1f}% command...", cmd);

        double velocity = measureSteadyStateVelocity(cmd, config_.velocity_test_duration);

        commands.push_back(cmd);
        velocities.push_back(velocity);

        LIBSTP_LOG_DEBUG("Measured velocity: {:.3f} rad/s", velocity);

        stopMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    // Linear regression: command = kS + kV * velocity
    // Fit y = slope * x + intercept where y=command, x=velocity
    // So: command = kV * velocity + kS_regression
    LinearRegression fit = linearFit(velocities, commands);

    result_.metrics.velocity_constant_r_squared = fit.r_squared;
    result_.metrics.velocity_samples = commands.size();

    double kV = fit.slope;
    double kS_regression = fit.intercept;

    LIBSTP_LOG_INFO("Velocity constant linear fit: kV={:.3f}, kS_from_regression={:.2f}%, R²={:.3f}",
                    kV, kS_regression, fit.r_squared);

    if (fit.r_squared < 0.8) {
        LIBSTP_LOG_WARN("Poor linear fit (R²={:.3f}), velocity constant may be inaccurate", fit.r_squared);
    }

    // Validate
    if (kV < config_.ranges.kV_min || kV > config_.ranges.kV_max) {
        LIBSTP_LOG_WARN("Velocity constant {:.3f} outside expected range [{:.3f}, {:.3f}]",
                    kV, config_.ranges.kV_min, config_.ranges.kV_max);
    }

    // Note: We're now using the regression intercept as kS instead of the threshold-based kS
    // This ensures consistency between kS and kV in the feedforward model
    // Update the stored static friction values to reflect the regression intercept
    result_.metrics.static_friction_forward = kS_regression;
    result_.metrics.static_friction_backward = kS_regression;

    return kV;
}

double MotorCalibrator::findAccelerationConstant(double kS, double kV)
{
    const double start_time = getCurrentTime();
    std::vector<double> kA_measurements;

    for (int trial = 0; trial < config_.acceleration_test_count; ++trial) {
        if (checkTimeout(start_time)) break;

        LIBSTP_LOG_DEBUG("Acceleration test trial {}/{}...", trial + 1, config_.acceleration_test_count);

        resetMotor();

        // Record velocity profile during step input
        VelocityProfile profile = recordVelocityProfile(
            config_.acceleration_test_amplitude,
            1.0  // 1 second duration
        );

        // Calculate acceleration from velocity change
        if (profile.data.size() < 10) {
            LIBSTP_LOG_WARN("Insufficient data points in acceleration test");
            continue;
        }

        // Use early portion of response (first 0.3 seconds) for acceleration when v ≈ 0
        // This is critical - we need early-time data where velocity is still small
        double max_time = profile.data.front().time + 0.3;
        std::vector<double> times, velocities;

        for (const auto& point : profile.data) {
            if (point.time <= max_time) {
                times.push_back(point.time - profile.data.front().time);
                velocities.push_back(point.velocity);
            }
        }

        if (times.size() < 5) continue;

        // Linear fit to get acceleration (slope)
        LinearRegression fit = linearFit(times, velocities);
        double acceleration = fit.slope;

        // Calculate kA from dynamics: command = kS + kV*v + kA*a
        // During early acceleration, v ≈ 0, so we ignore the kV term as stated in comments
        // Therefore: kA = (command - kS) / a
        // This implements exactly what the comment says without mixing in kV
        if (std::abs(acceleration) > 0.01) {
            double kA = (config_.acceleration_test_amplitude - kS) / acceleration;
            kA_measurements.push_back(kA);
            LIBSTP_LOG_DEBUG("Trial {}: acceleration={:.3f} rad/s², kA={:.3f}",
                        trial + 1, acceleration, kA);
        }

        stopMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    if (kA_measurements.empty()) {
        LIBSTP_LOG_WARN("No valid acceleration measurements, using default kA=0.1");
        return 0.1;
    }

    double kA = getMeanValue(kA_measurements);
    double kA_std = getStdDev(kA_measurements, kA);

    result_.metrics.acceleration_mean = kA;
    result_.metrics.acceleration_std_dev = kA_std;

    LIBSTP_LOG_INFO("Acceleration constant: kA={:.3f} ± {:.3f}", kA, kA_std);

    // Clamp to valid range to prevent negative or extreme values
    kA = clamp(kA, config_.ranges.kA_min, config_.ranges.kA_max);

    // Validate
    if (kA <= config_.ranges.kA_min || kA >= config_.ranges.kA_max) {
        LIBSTP_LOG_WARN("Acceleration constant clamped to range [{:.3f}, {:.3f}]",
                    config_.ranges.kA_min, config_.ranges.kA_max);
    }

    return kA;
}

foundation::PidGains MotorCalibrator::stepResponseTuning(const foundation::Feedforward& ff)
{
    LIBSTP_LOG_INFO("Performing step response test...");

    resetMotor();

    // Record step response
    VelocityProfile profile = recordVelocityProfile(
        config_.step_response_amplitude,
        config_.step_response_duration
    );

    stopMotor();

    if (profile.data.size() < 20) {
        LIBSTP_LOG_ERROR("Insufficient data for step response analysis");
        throw std::runtime_error("Step response test failed: insufficient data");
    }

    // Analyze step response
    double initial_velocity = profile.data.front().velocity;
    StepResponseParams params = fitStepResponse(profile, initial_velocity);

    result_.metrics.time_constant_tau = params.tau;
    result_.metrics.steady_state_gain = params.K;
    result_.metrics.delay = params.delay;

    LIBSTP_LOG_INFO("Step response parameters: tau={:.3f}s, K={:.3f}, delay={:.3f}s",
                params.tau, params.K, params.delay);

    // Calculate PID gains using FOPDT (First Order Plus Dead Time) model
    // Conservative tuning rules
    foundation::PidGains pid;

    if (params.delay > 0.001 && params.K > 0.001) {
        pid.kp = 0.6 * params.tau / (params.K * params.delay);
        pid.ki = pid.kp / (4.0 * params.delay);
        pid.kd = pid.kp * params.delay / 3.0;
    } else {
        // Fallback if parameters are invalid
        LIBSTP_LOG_WARN("Invalid step response parameters, using fallback gains");
        pid.kp = 2.0;
        pid.ki = 0.5;
        pid.kd = 0.1;
    }

    // Clamp to reasonable ranges
    pid.kp = clamp(pid.kp, config_.ranges.kp_min, config_.ranges.kp_max);
    pid.ki = clamp(pid.ki, config_.ranges.ki_min, config_.ranges.ki_max);
    pid.kd = clamp(pid.kd, config_.ranges.kd_min, config_.ranges.kd_max);

    LIBSTP_LOG_INFO("Calculated PID gains: kp={:.3f}, ki={:.3f}, kd={:.3f}",
                pid.kp, pid.ki, pid.kd);

    return pid;
}

foundation::PidGains MotorCalibrator::relayFeedbackTuning(const foundation::Feedforward& ff)
{
    LIBSTP_LOG_INFO("Performing relay feedback test...");

    resetMotor();

    // Record oscillatory response with relay control
    const double start_time = getCurrentTime();
    std::vector<VelocityDataPoint> data;

    bool relay_state = true;
    double relay_command = config_.relay_amplitude;
    double last_velocity = 0.0;
    int zero_crossings = 0;

    setMotorCommand(relay_command);

    while (getCurrentTime() - start_time < config_.max_relay_duration) {
        double current_time = getCurrentTime();
        double velocity = getMotorVelocity();

        data.push_back({current_time, velocity, relay_state ? relay_command : -relay_command});

        // Detect zero crossing (velocity changes sign relative to setpoint)
        if ((last_velocity > 0 && velocity < 0) || (last_velocity < 0 && velocity > 0)) {
            zero_crossings++;

            // Flip relay
            relay_state = !relay_state;
            relay_command = relay_state ? config_.relay_amplitude : -config_.relay_amplitude;
            setMotorCommand(relay_command);
        }

        last_velocity = velocity;

        // Need enough oscillations
        if (zero_crossings >= config_.min_oscillations * 2) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (checkTimeout(start_time)) {
            LIBSTP_LOG_ERROR("Relay feedback test timeout");
            break;
        }
    }

    stopMotor();

    VelocityProfile profile;
    profile.data = data;

    if (zero_crossings < config_.min_oscillations * 2) {
        LIBSTP_LOG_WARN("Insufficient oscillations ({}/{}), falling back to step response",
                    zero_crossings / 2, config_.min_oscillations);
        return stepResponseTuning(ff);
    }

    // Analyze oscillations
    RelayParams relay_params = analyzeOscillations(profile);

    result_.metrics.ultimate_gain_ku = relay_params.Ku;
    result_.metrics.ultimate_period_tu = relay_params.Tu;
    result_.metrics.oscillation_count = relay_params.cycles;

    LIBSTP_LOG_INFO("Relay parameters: Ku={:.3f}, Tu={:.3f}s, cycles={}",
                relay_params.Ku, relay_params.Tu, relay_params.cycles);

    // Calculate PID gains using Tyreus-Luyben rules (more conservative than Ziegler-Nichols)
    foundation::PidGains pid;

    if (relay_params.Tu > 0.001 && relay_params.Ku > 0.001) {
        pid.kp = relay_params.Ku / 2.2;
        pid.ki = pid.kp / (2.2 * relay_params.Tu);
        pid.kd = pid.kp * relay_params.Tu / 6.3;
    } else {
        LIBSTP_LOG_WARN("Invalid relay parameters, falling back to step response");
        return stepResponseTuning(ff);
    }

    // Clamp to reasonable ranges
    pid.kp = clamp(pid.kp, config_.ranges.kp_min, config_.ranges.kp_max);
    pid.ki = clamp(pid.ki, config_.ranges.ki_min, config_.ranges.ki_max);
    pid.kd = clamp(pid.kd, config_.ranges.kd_min, config_.ranges.kd_max);

    LIBSTP_LOG_INFO("Calculated PID gains (Tyreus-Luyben): kp={:.3f}, ki={:.3f}, kd={:.3f}",
                pid.kp, pid.ki, pid.kd);

    return pid;
}

// ============================================================================
// Data Recording and Analysis Functions
// ============================================================================

MotorCalibrator::VelocityProfile MotorCalibrator::recordVelocityProfile(
    double command_percent, double duration)
{
    VelocityProfile profile;
    const double start_time = getCurrentTime();
    double last_time = start_time;

    setMotorCommand(command_percent);

    while (getCurrentTime() - start_time < duration) {
        double current_time = getCurrentTime();
        double dt = current_time - last_time;
        last_time = current_time;

        // Update encoder velocity estimation
        if (dt > 0.0) {
            adapter_->updateEncoderVelocity(dt);
        }

        double velocity = getMotorVelocity();
        profile.data.push_back({current_time, velocity, command_percent});

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (checkTimeout(start_time) || emergency_stop_) {
            break;
        }
    }

    // Calculate statistics
    if (!profile.data.empty()) {
        std::vector<double> velocities;
        for (const auto& point : profile.data) {
            velocities.push_back(point.velocity);
        }
        profile.mean_velocity = getMeanValue(velocities);
        profile.std_dev = getStdDev(velocities, profile.mean_velocity);
    }

    return profile;
}

double MotorCalibrator::measureSteadyStateVelocity(double command_percent, double duration)
{
    VelocityProfile profile = recordVelocityProfile(command_percent, duration);

    // Use second half of data for steady-state measurement (after settling)
    size_t start_idx = profile.data.size() / 2;
    std::vector<double> steady_state_velocities;

    for (size_t i = start_idx; i < profile.data.size(); ++i) {
        steady_state_velocities.push_back(profile.data[i].velocity);
    }

    if (steady_state_velocities.empty()) {
        return 0.0;
    }

    return getMeanValue(steady_state_velocities);
}

MotorCalibrator::LinearRegression MotorCalibrator::linearFit(
    const std::vector<double>& x, const std::vector<double>& y)
{
    LinearRegression result{0.0, 0.0, 0.0};

    if (x.size() != y.size() || x.size() < 2) {
        return result;
    }

    const size_t n = x.size();
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;

    for (size_t i = 0; i < n; ++i) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_xx += x[i] * x[i];
    }

    double mean_x = sum_x / n;
    double mean_y = sum_y / n;

    double numerator = sum_xy - n * mean_x * mean_y;
    double denominator = sum_xx - n * mean_x * mean_x;

    if (std::abs(denominator) < 1e-10) {
        return result;
    }

    result.slope = numerator / denominator;
    result.intercept = mean_y - result.slope * mean_x;

    // Calculate R²
    double ss_tot = 0.0, ss_res = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double y_pred = result.slope * x[i] + result.intercept;
        ss_res += (y[i] - y_pred) * (y[i] - y_pred);
        ss_tot += (y[i] - mean_y) * (y[i] - mean_y);
    }

    result.r_squared = (ss_tot > 1e-10) ? 1.0 - (ss_res / ss_tot) : 0.0;

    return result;
}

MotorCalibrator::StepResponseParams MotorCalibrator::fitStepResponse(
    const VelocityProfile& profile, double initial_velocity)
{
    StepResponseParams params{0.0, 0.0, 0.0, 0.0};

    if (profile.data.size() < 10) {
        return params;
    }

    // Find steady-state velocity (last 20% of data)
    size_t start_ss = profile.data.size() * 4 / 5;
    std::vector<double> ss_velocities;
    for (size_t i = start_ss; i < profile.data.size(); ++i) {
        ss_velocities.push_back(profile.data[i].velocity);
    }
    double v_ss = getMeanValue(ss_velocities);

    // Steady-state gain K = Δvelocity / Δcommand
    // Use absolute value of steady-state velocity (ignore initial velocity which may be noisy)
    double v_change = v_ss;
    double command = std::abs(profile.data.front().command);

    if (command < 0.1) {
        LIBSTP_LOG_WARN("Step response command too small: {}", command);
        return params;
    }

    params.K = v_change / command;

    // Find time when velocity reaches 63.2% of final value (tau)
    double target_velocity = 0.632 * v_ss;
    double t0 = profile.data.front().time;

    for (const auto& point : profile.data) {
        if (point.velocity >= target_velocity) {
            params.tau = point.time - t0;
            break;
        }
    }

    // Estimate delay (when response starts to rise above 10% of final)
    double threshold = 0.1 * v_ss;
    for (const auto& point : profile.data) {
        if (point.velocity >= threshold) {
            params.delay = point.time - t0;
            break;
        }
    }

    // Default values if not found
    if (params.tau == 0.0) params.tau = 0.5;
    if (params.delay == 0.0) params.delay = 0.05;
    if (std::abs(params.K) < 0.001) params.K = 1.0;

    return params;
}

MotorCalibrator::RelayParams MotorCalibrator::analyzeOscillations(
    const VelocityProfile& profile)
{
    RelayParams params{0.0, 0.0, 0.0, 0};

    if (profile.data.size() < 10) {
        return params;
    }

    // Find zero crossings
    std::vector<double> crossing_times;
    double last_velocity = profile.data[0].velocity;

    for (size_t i = 1; i < profile.data.size(); ++i) {
        double v = profile.data[i].velocity;
        if ((last_velocity > 0 && v < 0) || (last_velocity < 0 && v > 0)) {
            crossing_times.push_back(profile.data[i].time);
        }
        last_velocity = v;
    }

    if (crossing_times.size() < 4) {
        return params;
    }

    // Calculate period (average time between alternate crossings)
    std::vector<double> periods;
    for (size_t i = 2; i < crossing_times.size(); ++i) {
        periods.push_back(crossing_times[i] - crossing_times[i-2]);
    }
    params.Tu = getMeanValue(periods);
    params.cycles = crossing_times.size() / 2;

    // Calculate amplitude (peak-to-peak velocity)
    double max_v = profile.data[0].velocity;
    double min_v = profile.data[0].velocity;
    for (const auto& point : profile.data) {
        max_v = std::max(max_v, point.velocity);
        min_v = std::min(min_v, point.velocity);
    }
    params.amplitude = (max_v - min_v) / 2.0;

    // Calculate ultimate gain Ku = 4*relay_amplitude / (π * oscillation_amplitude)
    double relay_amp = config_.relay_amplitude;
    params.Ku = (4.0 * relay_amp) / (3.14159265359 * params.amplitude);

    return params;
}

bool MotorCalibrator::validateCalibration(
    const foundation::PidGains& pid,
    const foundation::Feedforward& ff,
    CalibrationResult::Metrics& metrics)
{
    LIBSTP_LOG_INFO("Validating calibration...");

    // Don't apply PID/FF for validation - we're testing the feedforward model accuracy
    // by measuring open-loop response and comparing to model prediction

    resetMotor();

    // Test feedforward model at multiple command levels (in %)
    // Use lower test commands to stay in linear range
    std::vector<double> test_commands = {10.0, 15.0, 20.0};
    std::vector<double> errors;

    for (double cmd_percent : test_commands) {
        VelocityProfile profile = recordVelocityProfile(cmd_percent, config_.validation_duration);

        // Get steady-state velocity (last 30% of data)
        size_t start_idx = profile.data.size() * 7 / 10;
        std::vector<double> ss_velocities;
        for (size_t i = start_idx; i < profile.data.size(); ++i) {
            ss_velocities.push_back(profile.data[i].velocity);
        }

        if (ss_velocities.empty()) continue;

        double measured_v = getMeanValue(ss_velocities);

        // Predict velocity using feedforward model: cmd = kS + kV * v
        // Rearranged: v = (cmd - kS) / kV
        // Note: ff.kS and ff.kV are in normalized units [0-1], so we need to convert cmd_percent
        double cmd_normalized = cmd_percent / 100.0;
        double predicted_v = (cmd_normalized - ff.kS) / ff.kV;

        if (predicted_v > 0.1 && measured_v > 0.1) {
            double error = std::abs(measured_v - predicted_v) / measured_v;
            errors.push_back(error);
            LIBSTP_LOG_DEBUG("Validation cmd={:.1f}%: measured={:.2f} rad/s, predicted={:.2f} rad/s, error={:.1f}%",
                        cmd_percent, measured_v, predicted_v, error * 100.0);
        }

        stopMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (errors.empty()) {
        metrics.validation_passed = false;
        return false;
    }

    metrics.validation_mean_error = getMeanValue(errors);
    metrics.validation_max_error = *std::max_element(errors.begin(), errors.end());
    metrics.validation_passed = metrics.validation_max_error < config_.validation_max_error;

    LIBSTP_LOG_INFO("Validation: mean_error={:.1f}%, max_error={:.1f}%, passed={}",
                metrics.validation_mean_error * 100.0,
                metrics.validation_max_error * 100.0,
                metrics.validation_passed);

    return metrics.validation_passed;
}

// ============================================================================
// Safety and Utility Functions
// ============================================================================

bool MotorCalibrator::checkSafetyLimits()
{
    if (total_distance_moved_ > config_.max_test_distance_m) {
        LIBSTP_LOG_ERROR("Safety limit exceeded: distance={:.3f}m > max={:.3f}m",
                    total_distance_moved_, config_.max_test_distance_m);
        emergency_stop_ = true;
        stopMotor();
        return false;
    }
    return true;
}

bool MotorCalibrator::checkTimeout(double start_time)
{
    double elapsed = getCurrentTime() - start_time;

    if (elapsed > config_.max_single_test_duration) {
        LIBSTP_LOG_ERROR("Individual test timeout: {:.2f}s > {:.2f}s",
                    elapsed, config_.max_single_test_duration);
        return true;
    }

    double total_elapsed = getCurrentTime() - calibration_start_time_;
    if (total_elapsed > config_.max_calibration_duration) {
        LIBSTP_LOG_ERROR("Total calibration timeout: {:.2f}s > {:.2f}s",
                    total_elapsed, config_.max_calibration_duration);
        return true;
    }

    return false;
}

bool MotorCalibrator::validateParameterRanges(
    const foundation::PidGains& pid,
    const foundation::Feedforward& ff)
{
    bool valid = true;

    if (ff.kS < config_.ranges.kS_min || ff.kS > config_.ranges.kS_max) {
        LIBSTP_LOG_ERROR("kS={:.2f} out of range [{:.2f}, {:.2f}]",
                    ff.kS, config_.ranges.kS_min, config_.ranges.kS_max);
        valid = false;
    }

    if (ff.kV < config_.ranges.kV_min || ff.kV > config_.ranges.kV_max) {
        LIBSTP_LOG_ERROR("kV={:.3f} out of range [{:.3f}, {:.3f}]",
                    ff.kV, config_.ranges.kV_min, config_.ranges.kV_max);
        valid = false;
    }

    if (ff.kA < config_.ranges.kA_min || ff.kA > config_.ranges.kA_max) {
        LIBSTP_LOG_ERROR("kA={:.3f} out of range [{:.3f}, {:.3f}]",
                    ff.kA, config_.ranges.kA_min, config_.ranges.kA_max);
        valid = false;
    }

    if (pid.kp < config_.ranges.kp_min || pid.kp > config_.ranges.kp_max) {
        LIBSTP_LOG_ERROR("kp={:.3f} out of range [{:.3f}, {:.3f}]",
                    pid.kp, config_.ranges.kp_min, config_.ranges.kp_max);
        valid = false;
    }

    if (pid.ki < config_.ranges.ki_min || pid.ki > config_.ranges.ki_max) {
        LIBSTP_LOG_ERROR("ki={:.3f} out of range [{:.3f}, {:.3f}]",
                    pid.ki, config_.ranges.ki_min, config_.ranges.ki_max);
        valid = false;
    }

    if (pid.kd < config_.ranges.kd_min || pid.kd > config_.ranges.kd_max) {
        LIBSTP_LOG_ERROR("kd={:.3f} out of range [{:.3f}, {:.3f}]",
                    pid.kd, config_.ranges.kd_min, config_.ranges.kd_max);
        valid = false;
    }

    return valid;
}

void MotorCalibrator::resetMotor()
{
    stopMotor();

    // Wait for motor to fully stop - check velocity repeatedly
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        adapter_->updateEncoderVelocity(0.05);

        double velocity = std::abs(getMotorVelocity());
        if (velocity < 0.1) {
            break;  // Motor has stopped
        }
    }

    // Additional settling time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void MotorCalibrator::stopMotor()
{
    motor_.setSpeed(0);
}

double MotorCalibrator::getCurrentTime() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

double MotorCalibrator::getMotorVelocity() const
{
    return adapter_->getVelocity();
}

void MotorCalibrator::setMotorCommand(double percent)
{
    int command = static_cast<int>(std::round(clamp(percent, -100.0, 100.0)));
    motor_.setSpeed(command);
}
