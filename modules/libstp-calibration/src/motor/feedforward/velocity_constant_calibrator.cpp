#include "calibration/motor/feedforward/velocity_constant_calibrator.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/data/velocity_profile_recorder.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "foundation/logging.hpp"
#include <algorithm>
#include <chrono>
#include <thread>

namespace libstp::calibration::feedforward
{
    VelocityConstantCalibrator::VelocityConstantCalibrator(
        motor::MotorControlInterface& motor,
        data::VelocityProfileRecorder& recorder,
        const CalibrationConfig& config)
        : motor_(motor)
        , recorder_(recorder)
        , config_(config)
    {
    }

    double VelocityConstantCalibrator::findVelocityConstant(
        double& kS_from_regression,
        double& r_squared,
        int& sample_count,
        double start_time,
        bool& emergency_stop,
        double static_friction_offset)
    {
        std::vector<double> commands;
        std::vector<double> velocities;

        // Offset test commands by static friction to ensure motor actually moves
        double offset = std::max(0.0, static_friction_offset + 5.0); // Add 5% margin above static friction
        if (offset > 5.0) {
            LIBSTP_LOG_INFO("Applying {:.1f}% offset to velocity tests (static friction={:.1f}%)",
                        offset, static_friction_offset);
        }

        for (double base_cmd : config_.velocity_test_commands) {
            double cmd = base_cmd + offset;
            if ((utils::getCurrentTime() - start_time) > config_.max_single_test_duration)
                break;

            LIBSTP_LOG_DEBUG("Testing velocity at {:.1f}% command...", cmd);

            double velocity = recorder_.measureSteadyStateVelocity(
                cmd, config_.velocity_test_duration, emergency_stop, start_time);

            commands.push_back(cmd);
            velocities.push_back(velocity);

            LIBSTP_LOG_DEBUG("Measured velocity: {:.3f} rad/s", velocity);

            motor_.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }

        // Linear regression: command = kS + kV * velocity
        auto fit = utils::linearFit(velocities, commands);

        r_squared = fit.r_squared;
        sample_count = commands.size();

        double kV = fit.slope;
        kS_from_regression = fit.intercept;

        LIBSTP_LOG_INFO("Velocity constant linear fit: kV={:.3f}, kS_from_regression={:.2f}%, R²={:.3f}",
                    kV, kS_from_regression, r_squared);

        if (r_squared < 0.8) {
            LIBSTP_LOG_WARN("Poor linear fit (R²={:.3f}), velocity constant may be inaccurate", r_squared);
        }

        // Validate
        if (kV < config_.ranges.kV_min || kV > config_.ranges.kV_max) {
            LIBSTP_LOG_WARN("Velocity constant {:.3f} outside expected range [{:.3f}, {:.3f}]",
                        kV, config_.ranges.kV_min, config_.ranges.kV_max);
        }

        return kV;
    }
}
