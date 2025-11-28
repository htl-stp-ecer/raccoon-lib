#include "motion/motion_pid.hpp"
#include <algorithm>
#include <cmath>

namespace libstp::motion
{
    MotionPidController::MotionPidController(Config config)
        : cfg_(config)
    {
    }

    double MotionPidController::update(double error, double dt)
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
        double derivative = 0.0;
        if (!first_update_)
        {
            const double raw_derivative = (error - prev_error_) / dt;
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

        // Store error for next iteration
        prev_error_ = error;

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
