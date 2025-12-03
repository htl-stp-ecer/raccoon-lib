#include "calibration/motor/feedforward/static_friction_calibrator.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "foundation/logging.hpp"
#include <chrono>
#include <thread>
#include <cmath>

namespace libstp::calibration::feedforward
{
    StaticFrictionCalibrator::StaticFrictionCalibrator(
        motor::MotorControlInterface& motor,
        const CalibrationConfig& config)
        : motor_(motor)
        , config_(config)
    {
    }

    double StaticFrictionCalibrator::findStaticFriction(
        double& forward_threshold,
        double& backward_threshold,
        double start_time)
    {
        const double test_start = utils::getCurrentTime();

        // Test forward direction
        LIBSTP_LOG_DEBUG("Testing forward static friction...");
        motor_.reset();

        forward_threshold = 0.0;
        for (double cmd = 0.0; cmd <= config_.static_friction_max; cmd += config_.static_friction_increment) {
            motor_.setCommand(cmd);

            // Wait for motor to respond and update velocity estimate
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            motor_.updateEncoderVelocity(0.05);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            motor_.updateEncoderVelocity(0.1);

            double velocity = std::abs(motor_.getVelocity());
            LIBSTP_LOG_DEBUG("Forward cmd={:.2f}%, velocity={:.3f} rad/s", cmd, velocity);

            if (velocity > config_.min_velocity_threshold) {
                forward_threshold = cmd;
                LIBSTP_LOG_DEBUG("Forward threshold found at {:.2f}%", cmd);
                break;
            }

            if ((utils::getCurrentTime() - start_time) > config_.max_single_test_duration)
                break;
        }

        motor_.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Test backward direction
        LIBSTP_LOG_DEBUG("Testing backward static friction...");
        motor_.reset();

        backward_threshold = 0.0;
        for (double cmd = 0.0; cmd <= config_.static_friction_max; cmd += config_.static_friction_increment) {
            motor_.setCommand(-cmd);

            // Wait for motor to respond and update velocity estimate
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            motor_.updateEncoderVelocity(0.05);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            motor_.updateEncoderVelocity(0.1);

            double velocity = std::abs(motor_.getVelocity());
            LIBSTP_LOG_DEBUG("Backward cmd={:.2f}%, velocity={:.3f} rad/s", -cmd, velocity);

            if (velocity > config_.min_velocity_threshold) {
                backward_threshold = cmd;
                LIBSTP_LOG_DEBUG("Backward threshold found at {:.2f}%", cmd);
                break;
            }

            if ((utils::getCurrentTime() - start_time) > config_.max_single_test_duration)
                break;
        }

        motor_.stop();

        // Return average
        double kS = (forward_threshold + backward_threshold) / 2.0;

        // Validate
        if (kS < config_.ranges.kS_min || kS > config_.ranges.kS_max) {
            LIBSTP_LOG_WARN("Static friction {:.2f} outside expected range [{:.2f}, {:.2f}]",
                        kS, config_.ranges.kS_min, config_.ranges.kS_max);
        }

        return kS;
    }
}
