#include "calibration/motor/pid/step_response_tuner.hpp"
#include "calibration/motor/control/motor_control_interface.hpp"
#include "calibration/motor/data/velocity_profile_recorder.hpp"
#include "calibration/motor/analysis/step_response_analyzer.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include "foundation/logging.hpp"
#include <stdexcept>

namespace libstp::calibration::pid
{
    StepResponseTuner::StepResponseTuner(
        motor::MotorControlInterface& motor,
        data::VelocityProfileRecorder& recorder,
        const CalibrationConfig& config)
        : motor_(motor)
        , recorder_(recorder)
        , config_(config)
    {
    }

    foundation::PidGains StepResponseTuner::tune(
        const foundation::Feedforward& ff,
        double& tau,
        double& steady_state_gain,
        double& delay,
        double start_time,
        bool& emergency_stop)
    {
        LIBSTP_LOG_INFO("Performing step response test...");

        motor_.reset();

        // Record step response
        auto profile = recorder_.recordProfile(
            config_.step_response_amplitude,
            config_.step_response_duration,
            emergency_stop,
            start_time
        );

        motor_.stop();

        if (profile.data.size() < 20) {
            LIBSTP_LOG_ERROR("Insufficient data for step response analysis");
            throw std::runtime_error("Step response test failed: insufficient data");
        }

        // Analyze step response
        double initial_velocity = profile.data.front().velocity;
        auto params = analysis::fitStepResponse(profile, initial_velocity);

        tau = params.tau;
        steady_state_gain = params.K;
        delay = params.delay;

        LIBSTP_LOG_INFO("Step response parameters: tau={:.3f}s, K={:.3f}, delay={:.3f}s",
                    params.tau, params.K, params.delay);

        // Calculate PID gains using FOPDT model
        foundation::PidGains pid;

        if (params.delay > 0.001 && params.K > 0.001) {
            pid.kp = 0.6 * params.tau / (params.K * params.delay);
            pid.ki = pid.kp / (4.0 * params.delay);
            pid.kd = pid.kp * params.delay / 3.0;
        } else {
            LIBSTP_LOG_WARN("Invalid step response parameters, using fallback gains");
            pid.kp = 2.0;
            pid.ki = 0.5;
            pid.kd = 0.1;
        }

        // Clamp to reasonable ranges
        pid.kp = utils::clamp(pid.kp, config_.ranges.kp_min, config_.ranges.kp_max);
        pid.ki = utils::clamp(pid.ki, config_.ranges.ki_min, config_.ranges.ki_max);
        pid.kd = utils::clamp(pid.kd, config_.ranges.kd_min, config_.ranges.kd_max);

        LIBSTP_LOG_INFO("Calculated PID gains: kp={:.3f}, ki={:.3f}, kd={:.3f}",
                    pid.kp, pid.ki, pid.kd);

        return pid;
    }
}
