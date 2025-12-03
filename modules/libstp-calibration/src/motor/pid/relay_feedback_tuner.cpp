#include "calibration/motor/pid/relay_feedback_tuner.hpp"
#include "calibration/motor/pid/step_response_tuner.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/data/velocity_data.hpp"
#include "calibration/motor/analysis/relay_oscillation_analyzer.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "foundation/logging.hpp"
#include <chrono>
#include <thread>

namespace libstp::calibration::pid
{
    RelayFeedbackTuner::RelayFeedbackTuner(
        motor::MotorControlInterface& motor,
        StepResponseTuner& step_tuner,
        const CalibrationConfig& config)
        : motor_(motor)
        , step_tuner_(step_tuner)
        , config_(config)
    {
    }

    foundation::PidGains RelayFeedbackTuner::tune(
        const foundation::Feedforward& ff,
        double& ultimate_gain,
        double& ultimate_period,
        int& oscillation_count,
        double start_time,
        bool& emergency_stop)
    {
        LIBSTP_LOG_INFO("Performing relay feedback test...");

        motor_.reset();

        // Record oscillatory response with relay control
        const double relay_start = utils::getCurrentTime();
        std::vector<data::VelocityDataPoint> data;

        bool relay_state = true;
        double relay_command = config_.relay_amplitude;
        double last_velocity = 0.0;
        int zero_crossings = 0;

        motor_.setCommand(relay_command);

        while (utils::getCurrentTime() - relay_start < config_.max_relay_duration) {
            double current_time = utils::getCurrentTime();
            double velocity = motor_.getVelocity();

            data.push_back({current_time, velocity, relay_state ? relay_command : -relay_command});

            // Detect zero crossing
            if ((last_velocity > 0 && velocity < 0) || (last_velocity < 0 && velocity > 0)) {
                zero_crossings++;

                // Flip relay
                relay_state = !relay_state;
                relay_command = relay_state ? config_.relay_amplitude : -config_.relay_amplitude;
                motor_.setCommand(relay_command);
            }

            last_velocity = velocity;

            if (zero_crossings >= config_.min_oscillations * 2) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if ((utils::getCurrentTime() - start_time) > config_.max_single_test_duration) {
                LIBSTP_LOG_ERROR("Relay feedback test timeout");
                break;
            }
        }

        motor_.stop();

        data::VelocityProfile profile;
        profile.data = data;

        if (zero_crossings < config_.min_oscillations * 2) {
            LIBSTP_LOG_WARN("Insufficient oscillations ({}/{}), falling back to step response",
                        zero_crossings / 2, config_.min_oscillations);
            double tau, K, delay;
            return step_tuner_.tune(ff, tau, K, delay, start_time, emergency_stop);
        }

        // Analyze oscillations
        auto relay_params = analysis::analyzeOscillations(profile, config_.relay_amplitude);

        ultimate_gain = relay_params.Ku;
        ultimate_period = relay_params.Tu;
        oscillation_count = relay_params.cycles;

        LIBSTP_LOG_INFO("Relay parameters: Ku={:.3f}, Tu={:.3f}s, cycles={}",
                    relay_params.Ku, relay_params.Tu, relay_params.cycles);

        // Calculate PID gains using Tyreus-Luyben rules
        foundation::PidGains pid;

        if (relay_params.Tu > 0.001 && relay_params.Ku > 0.001) {
            pid.kp = relay_params.Ku / 2.2;
            pid.ki = pid.kp / (2.2 * relay_params.Tu);
            pid.kd = pid.kp * relay_params.Tu / 6.3;
        } else {
            LIBSTP_LOG_WARN("Invalid relay parameters, falling back to step response");
            double tau, K, delay;
            return step_tuner_.tune(ff, tau, K, delay, start_time, emergency_stop);
        }

        // Clamp to reasonable ranges
        pid.kp = utils::clamp(pid.kp, config_.ranges.kp_min, config_.ranges.kp_max);
        pid.ki = utils::clamp(pid.ki, config_.ranges.ki_min, config_.ranges.ki_max);
        pid.kd = utils::clamp(pid.kd, config_.ranges.kd_min, config_.ranges.kd_max);

        LIBSTP_LOG_INFO("Calculated PID gains (Tyreus-Luyben): kp={:.3f}, ki={:.3f}, kd={:.3f}",
                    pid.kp, pid.ki, pid.kd);

        return pid;
    }
}
