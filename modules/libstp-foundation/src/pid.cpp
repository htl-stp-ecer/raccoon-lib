#include "foundation/pid.hpp"
#include <algorithm>
#include <cmath>

namespace libstp::foundation
{
    PidController::PidController(PidConfig config)
        : cfg_(config)
    {
    }

    double PidController::update(double error, double dt)
    {
        return update(error, dt, error);
    }

    double PidController::update(double error, double dt, double deriv_signal)
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
            integral_ = std::clamp(integral_, -cfg_.integral_max, cfg_.integral_max);
        }
        const double i_term = cfg_.ki * integral_;

        // Derivative term with low-pass filtering
        double derivative = 0.0;
        if (!first_update_)
        {
            const double raw_derivative = (deriv_signal - prev_error_) / dt;
            const double alpha = std::clamp(cfg_.derivative_lpf_alpha, 0.0, 1.0);
            filtered_derivative_ = alpha * raw_derivative + (1.0 - alpha) * filtered_derivative_;
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

    void PidController::reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        filtered_derivative_ = 0.0;
        first_update_ = true;
    }

    void PidController::setGains(double kp, double ki, double kd)
    {
        cfg_.kp = kp;
        cfg_.ki = ki;
        cfg_.kd = kd;
    }
}
