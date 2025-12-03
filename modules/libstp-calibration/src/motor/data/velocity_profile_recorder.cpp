#include "calibration/motor/data/velocity_profile_recorder.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include <chrono>
#include <thread>

namespace libstp::calibration::data
{
    VelocityProfileRecorder::VelocityProfileRecorder(motor::MotorControlInterface& motor)
        : motor_(motor)
    {
    }

    VelocityProfile VelocityProfileRecorder::recordProfile(
        double command_percent,
        double duration,
        bool& emergency_stop,
        double start_time)
    {
        VelocityProfile profile;
        const double record_start = utils::getCurrentTime();
        double last_time = record_start;

        motor_.setCommand(command_percent);

        while (utils::getCurrentTime() - record_start < duration) {
            double current_time = utils::getCurrentTime();
            double dt = current_time - last_time;
            last_time = current_time;

            // Update encoder velocity estimation
            if (dt > 0.0) {
                motor_.updateEncoderVelocity(dt);
            }

            double velocity = motor_.getVelocity();
            profile.data.push_back({current_time, velocity, command_percent});

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if ((utils::getCurrentTime() - start_time) > 30.0 || emergency_stop) {
                break;
            }
        }

        // Calculate statistics
        if (!profile.data.empty()) {
            std::vector<double> velocities;
            for (const auto& point : profile.data) {
                velocities.push_back(point.velocity);
            }
            profile.mean_velocity = utils::getMeanValue(velocities);
            profile.std_dev = utils::getStdDev(velocities, profile.mean_velocity);
        }

        return profile;
    }

    double VelocityProfileRecorder::measureSteadyStateVelocity(
        double command_percent,
        double duration,
        bool& emergency_stop,
        double start_time)
    {
        VelocityProfile profile = recordProfile(command_percent, duration, emergency_stop, start_time);

        // Use second half of data for steady-state measurement (after settling)
        size_t start_idx = profile.data.size() / 2;
        std::vector<double> steady_state_velocities;

        for (size_t i = start_idx; i < profile.data.size(); ++i) {
            steady_state_velocities.push_back(profile.data[i].velocity);
        }

        if (steady_state_velocities.empty()) {
            return 0.0;
        }

        return utils::getMeanValue(steady_state_velocities);
    }
}
