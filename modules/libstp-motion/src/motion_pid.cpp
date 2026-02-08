#include "motion/motion_pid.hpp"
#include "motion/motion_config.hpp"
#include <algorithm>
#include <cmath>

namespace libstp::motion
{
    std::unique_ptr<MotionPidController> createPidController(
        const UnifiedMotionPidConfig& unified_config,
        PidType type)
    {
        MotionPidController::Config cfg;

        // Set type-specific gains
        switch (type)
        {
        case PidType::Distance:
            cfg.kp = unified_config.distance_kp;
            cfg.ki = unified_config.distance_ki;
            cfg.kd = unified_config.distance_kd;
            break;
        case PidType::Heading:
            cfg.kp = unified_config.heading_kp;
            cfg.ki = unified_config.heading_ki;
            cfg.kd = unified_config.heading_kd;
            break;
        case PidType::Lateral:
            cfg.kp = unified_config.lateral_kp;
            cfg.ki = unified_config.lateral_ki;
            cfg.kd = unified_config.lateral_kd;
            break;
        }

        // Set shared parameters
        cfg.output_min = unified_config.output_min;
        cfg.output_max = unified_config.output_max;
        cfg.integral_max = unified_config.integral_max;
        cfg.integral_deadband = unified_config.integral_deadband;
        cfg.derivative_lpf_alpha = unified_config.derivative_lpf_alpha;

        return std::make_unique<MotionPidController>(cfg);
    }

    MotionPidController::MotionPidController(Config config)
        : cfg_(config)
    {
    }

    double MotionPidController::update(double error, double dt)
    {
        return update(error, dt, error);
    }

    double MotionPidController::update(double error, double dt, double deriv_signal)
    {
        if (dt <= 0.0)
        {
            return 0.0;
        }

        // Proportional term
        const double p_term = cfg_.kp * error;

        // Integral term with anti-windup
        if (std::abs(error) > cfg_.integral_deadband)
        {
            integral_ += error * dt;
            // Clamp integral
            integral_ = std::clamp(integral_, -cfg_.integral_max, cfg_.integral_max);
        }
        const double i_term = cfg_.ki * integral_;

        // Derivative term with low-pass filtering
        // Uses deriv_signal (which may differ from error, e.g. unclamped error)
        // so the D term can see the true rate of change even when error is clamped.
        double derivative = 0.0;
        if (!first_update_)
        {
            const double raw_derivative = (deriv_signal - prev_error_) / dt;
            // Low-pass filter: d_filt_new = alpha * d_raw + (1-alpha) * d_filt_old
            filtered_derivative_ = cfg_.derivative_lpf_alpha * raw_derivative +
                                  (1.0 - cfg_.derivative_lpf_alpha) * filtered_derivative_;
            derivative = filtered_derivative_;
        }
        else
        {
            first_update_ = false;
        }
        const double d_term = cfg_.kd * derivative;

        // Compute total output
        double output = p_term + i_term + d_term;

        // Apply output saturation
        output = std::clamp(output, cfg_.output_min, cfg_.output_max);

        // Store deriv_signal for next derivative calculation
        prev_error_ = deriv_signal;

        return output;
    }

    void MotionPidController::reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        filtered_derivative_ = 0.0;
        first_update_ = true;
    }

    void MotionPidController::setGains(double kp, double ki, double kd)
    {
        cfg_.kp = kp;
        cfg_.ki = ki;
        cfg_.kd = kd;
    }
}
