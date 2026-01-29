#include "calibration/motor/validation/calibration_validator.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/data/velocity_profile_recorder.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include "foundation/logging.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>

namespace libstp::calibration::validation
{
    CalibrationValidator::CalibrationValidator(
        motor::MotorControlInterface& motor,
        data::VelocityProfileRecorder& recorder,
        const CalibrationConfig& config)
        : motor_(motor)
        , recorder_(recorder)
        , config_(config)
    {
    }

    bool CalibrationValidator::validateCalibration(
        const foundation::PidGains& pid,
        const foundation::Feedforward& ff,
        CalibrationResult::Metrics& metrics,
        double start_time,
        bool& emergency_stop)
    {
        LIBSTP_LOG_INFO("Validating calibration...");

        motor_.reset();

        std::vector<double> test_commands = {10.0, 15.0, 20.0};
        std::vector<double> errors;

        for (double cmd_percent : test_commands) {
            auto profile = recorder_.recordProfile(
                cmd_percent,
                config_.validation_duration,
                emergency_stop,
                start_time
            );

            // Get steady-state velocity (last 30% of data)
            size_t start_idx = profile.data.size() * 7 / 10;
            std::vector<double> ss_velocities;
            for (size_t i = start_idx; i < profile.data.size(); ++i) {
                ss_velocities.push_back(profile.data[i].velocity);
            }

            if (ss_velocities.empty()) continue;

            double measured_v = utils::getMeanValue(ss_velocities);

            // Predict velocity using feedforward model
            double cmd_normalized = cmd_percent / 100.0;
            double predicted_v = (cmd_normalized - ff.kS) / ff.kV;

            if (predicted_v > 0.1 && measured_v > 0.1) {
                double error = std::abs(measured_v - predicted_v) / measured_v;
                errors.push_back(error);
                LIBSTP_LOG_DEBUG("Validation cmd={:.1f}%: measured={:.2f} rad/s, predicted={:.2f} rad/s, error={:.1f}%",
                            cmd_percent, measured_v, predicted_v, error * 100.0);
            }

            motor_.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        if (errors.empty()) {
            metrics.validation_passed = false;
            return false;
        }

        metrics.validation_mean_error = utils::getMeanValue(errors);
        metrics.validation_max_error = *std::max_element(errors.begin(), errors.end());
        metrics.validation_passed = metrics.validation_max_error < config_.validation_max_error;

        LIBSTP_LOG_INFO("Validation: mean_error={:.1f}%, max_error={:.1f}%, passed={}",
                    metrics.validation_mean_error * 100.0,
                    metrics.validation_max_error * 100.0,
                    metrics.validation_passed);

        return metrics.validation_passed;
    }

    bool CalibrationValidator::validateParameterRanges(
        const foundation::PidGains& pid,
        const foundation::Feedforward& ff)
    {
        if (!config_.validate_parameter_ranges) {
            LIBSTP_LOG_INFO("Parameter range validation disabled");
            return true;
        }

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
}
