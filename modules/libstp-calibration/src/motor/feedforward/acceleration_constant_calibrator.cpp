#include "calibration/motor/feedforward/acceleration_constant_calibrator.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/data/velocity_profile_recorder.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "foundation/logging.hpp"
#include <chrono>
#include <thread>
#include <cmath>

namespace libstp::calibration::feedforward
{
    AccelerationConstantCalibrator::AccelerationConstantCalibrator(
        motor::MotorControlInterface& motor,
        data::VelocityProfileRecorder& recorder,
        const CalibrationConfig& config)
        : motor_(motor)
        , recorder_(recorder)
        , config_(config)
    {
    }

    double AccelerationConstantCalibrator::findAccelerationConstant(
        double kS,
        double kV,
        double& mean_value,
        double& std_dev,
        double start_time,
        bool& emergency_stop)
    {
        std::vector<double> kA_measurements;

        for (int trial = 0; trial < config_.acceleration_test_count; ++trial) {
            if ((utils::getCurrentTime() - start_time) > config_.max_single_test_duration)
                break;

            LIBSTP_LOG_DEBUG("Acceleration test trial {}/{}...", trial + 1, config_.acceleration_test_count);

            motor_.reset();

            // Record velocity profile during step input
            auto profile = recorder_.recordProfile(
                config_.acceleration_test_amplitude,
                1.0,
                emergency_stop,
                start_time
            );

            // Calculate acceleration from velocity change
            if (profile.data.size() < 10) {
                LIBSTP_LOG_WARN("Insufficient data points in acceleration test");
                continue;
            }

            // Use early portion of response (first 0.3 seconds)
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
            auto fit = utils::linearFit(times, velocities);
            double acceleration = fit.slope;

            // Calculate kA from dynamics: command = kS + kV*v + kA*a
            // During early acceleration, v ≈ 0, so: kA = (command - kS) / a
            if (std::abs(acceleration) > 0.01) {
                double kA = (config_.acceleration_test_amplitude - kS) / acceleration;
                kA_measurements.push_back(kA);
                LIBSTP_LOG_DEBUG("Trial {}: acceleration={:.3f} rad/s², kA={:.3f}",
                            trial + 1, acceleration, kA);
            }

            motor_.stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }

        if (kA_measurements.empty()) {
            LIBSTP_LOG_WARN("No valid acceleration measurements, using default kA=0.1");
            mean_value = 0.1;
            std_dev = 0.0;
            return 0.1;
        }

        double kA = utils::getMeanValue(kA_measurements);
        double kA_std = utils::getStdDev(kA_measurements, kA);

        mean_value = kA;
        std_dev = kA_std;

        LIBSTP_LOG_INFO("Acceleration constant: kA={:.3f} ± {:.3f}", kA, kA_std);

        // Clamp to valid range
        kA = utils::clamp(kA, config_.ranges.kA_min, config_.ranges.kA_max);

        // Validate
        if (kA <= config_.ranges.kA_min || kA >= config_.ranges.kA_max) {
            LIBSTP_LOG_WARN("Acceleration constant clamped to range [{:.3f}, {:.3f}]",
                        config_.ranges.kA_min, config_.ranges.kA_max);
        }

        return kA;
    }
}
