#include "calibration/motor/calibration.hpp"

// Include all component headers
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/data/velocity_profile_recorder.hpp"
#include "calibration/motor/feedforward/static_friction_calibrator.hpp"
#include "calibration/motor/feedforward/velocity_constant_calibrator.hpp"
#include "calibration/motor/feedforward/acceleration_constant_calibrator.hpp"
#include "calibration/motor/pid/step_response_tuner.hpp"
#include "calibration/motor/pid/relay_feedback_tuner.hpp"
#include "calibration/motor/validation/calibration_validator.hpp"
#include "calibration/motor/validation/safety_monitor.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "drive/motor_adapter.hpp"
#include "foundation/logging.hpp"

namespace libstp::calibration
{

MotorCalibrator::MotorCalibrator(hal::motor::Motor& motor, CalibrationConfig config)
    : config_(std::move(config))
{
    LIBSTP_LOG_TRACE("MotorCalibrator created for motor port {}", motor.port);

    // Create motor control interface
    motor_control_ = std::make_unique<motor::MotorControlInterface>(motor);

    // Create data recorder
    recorder_ = std::make_unique<data::VelocityProfileRecorder>(*motor_control_);

    // Create feedforward calibrators
    static_friction_cal_ = std::make_unique<feedforward::StaticFrictionCalibrator>(
        *motor_control_, config_);
    velocity_cal_ = std::make_unique<feedforward::VelocityConstantCalibrator>(
        *motor_control_, *recorder_, config_);
    acceleration_cal_ = std::make_unique<feedforward::AccelerationConstantCalibrator>(
        *motor_control_, *recorder_, config_);

    // Create PID tuners
    step_tuner_ = std::make_unique<pid::StepResponseTuner>(
        *motor_control_, *recorder_, config_);
    relay_tuner_ = std::make_unique<pid::RelayFeedbackTuner>(
        *motor_control_, *step_tuner_, config_);

    // Create validation components
    validator_ = std::make_unique<validation::CalibrationValidator>(
        *motor_control_, *recorder_, config_);
    safety_monitor_ = std::make_unique<validation::SafetyMonitor>(config_);
}

MotorCalibrator::~MotorCalibrator() = default;

CalibrationResult MotorCalibrator::calibrate()
{
    LIBSTP_LOG_INFO("=== Starting motor calibration ===");
    calibration_start_time_ = utils::getCurrentTime();
    result_ = CalibrationResult{};
    result_.success = false;
    emergency_stop_ = false;
    total_distance_moved_ = 0.0;

    try {
        // Step 1: Calibrate feedforward
        LIBSTP_LOG_INFO("Step 1/3: Calibrating feedforward parameters...");
        result_.ff = calibrateFeedforward();

        if (safety_monitor_->checkTimeout(calibration_start_time_, calibration_start_time_)) {
            result_.error_message = "Calibration timeout during feedforward calibration";
            LIBSTP_LOG_ERROR("{}", result_.error_message);
            return result_;
        }

        // Step 2: Calibrate PID using the measured feedforward
        LIBSTP_LOG_INFO("Step 2/3: Calibrating PID parameters...");
        result_.pid = calibratePID(result_.ff);

        if (safety_monitor_->checkTimeout(calibration_start_time_, calibration_start_time_)) {
            result_.error_message = "Calibration timeout during PID calibration";
            LIBSTP_LOG_ERROR("{}", result_.error_message);
            return result_;
        }

        // Step 3: Validate the calibration
        LIBSTP_LOG_INFO("Step 3/3: Validating calibration...");
        if (!validator_->validateCalibration(result_.pid, result_.ff, result_.metrics,
                                             calibration_start_time_, emergency_stop_)) {
            result_.error_message = "Calibration validation failed";
            LIBSTP_LOG_WARN("{}", result_.error_message);
        }

        // Check parameter ranges
        if (!validator_->validateParameterRanges(result_.pid, result_.ff)) {
            result_.error_message = "Calibrated parameters out of acceptable range";
            LIBSTP_LOG_ERROR("{}", result_.error_message);
            return result_;
        }

        result_.duration_seconds = utils::getCurrentTime() - calibration_start_time_;
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
        motor_control_->stop();
    }

    motor_control_->stop();
    return result_;
}

foundation::Feedforward MotorCalibrator::calibrateFeedforward()
{
    foundation::Feedforward ff;

    // Find static friction threshold (for reference/debugging only)
    LIBSTP_LOG_INFO("Finding static friction threshold (for reference)...");
    double forward_threshold, backward_threshold;
    double kS_threshold = static_friction_cal_->findStaticFriction(
        forward_threshold, backward_threshold, calibration_start_time_);
    LIBSTP_LOG_INFO("Static friction threshold = {:.2f}%", kS_threshold);

    // Find velocity constant (kV) and kS from linear regression
    LIBSTP_LOG_INFO("Finding velocity constant via linear regression...");
    double kS_percent, r_squared;
    int sample_count;
    double kV_percent = velocity_cal_->findVelocityConstant(
        kS_percent, r_squared, sample_count, calibration_start_time_, emergency_stop_);

    // Store metrics
    result_.metrics.static_friction_forward = kS_percent;
    result_.metrics.static_friction_backward = kS_percent;
    result_.metrics.velocity_constant_r_squared = r_squared;
    result_.metrics.velocity_samples = sample_count;

    LIBSTP_LOG_INFO("Linear regression results: kS={:.2f}%, kV={:.3f}% per rad/s", kS_percent, kV_percent);

    // Find acceleration constant (kA)
    LIBSTP_LOG_INFO("Finding acceleration constant...");
    double kA_mean, kA_std;
    double kA_percent = acceleration_cal_->findAccelerationConstant(
        kS_percent, kV_percent, kA_mean, kA_std, calibration_start_time_, emergency_stop_);

    result_.metrics.acceleration_mean = kA_mean;
    result_.metrics.acceleration_std_dev = kA_std;

    LIBSTP_LOG_INFO("Acceleration constant kA = {:.3f}% per rad/s²", kA_percent);

    // Convert from percent [0-100] to normalized [0-1]
    ff.kS = kS_percent / 100.0;
    ff.kV = kV_percent / 100.0;
    ff.kA = kA_percent / 100.0;

    LIBSTP_LOG_INFO("Converted to normalized units: kS={:.4f}, kV={:.4f}, kA={:.4f}", ff.kS, ff.kV, ff.kA);

    return ff;
}

foundation::PidGains MotorCalibrator::calibratePID(const foundation::Feedforward& ff)
{
    // Apply feedforward to the controller for PID tuning
    motor_control_->getAdapter().getController().setFF(ff);

    if (config_.use_relay_feedback) {
        LIBSTP_LOG_INFO("Using relay feedback tuning (aggressive mode)");
        double Ku, Tu;
        int osc_count;
        auto pid = relay_tuner_->tune(ff, Ku, Tu, osc_count, calibration_start_time_, emergency_stop_);

        result_.metrics.ultimate_gain_ku = Ku;
        result_.metrics.ultimate_period_tu = Tu;
        result_.metrics.oscillation_count = osc_count;

        return pid;
    } else {
        LIBSTP_LOG_INFO("Using step response tuning (conservative mode)");
        double tau, K, delay;
        auto pid = step_tuner_->tune(ff, tau, K, delay, calibration_start_time_, emergency_stop_);

        result_.metrics.time_constant_tau = tau;
        result_.metrics.steady_state_gain = K;
        result_.metrics.delay = delay;

        return pid;
    }
}

} // namespace libstp::calibration
