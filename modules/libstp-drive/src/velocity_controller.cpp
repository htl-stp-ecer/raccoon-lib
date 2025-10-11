//
// Created by tobias on 9/8/25.
//
#include "drive/velocity_controller.hpp"
#include <cmath>
#include <algorithm>

using namespace libstp::drive;

VelocityController::VelocityController(foundation::PidGains g, foundation::Feedforward ff)
    : g_(g), ff_(ff)
{
}

void VelocityController::setGains(const foundation::PidGains& g) { g_ = g; }
void VelocityController::setFF(const foundation::Feedforward& ff) { ff_ = ff; }

double VelocityController::compute(double w_ref, double a_ref, double w_meas, double dt,
                                   double u_max, bool* out_sat)
{
    const double d_meas = (dt > 0.0) ? (w_meas - w_meas_prev_) / dt : 0.0;
    w_meas_prev_ = w_meas;
    d_filt_ += d_alpha_ * (d_meas - d_filt_);

    const double u_ff = (w_ref != 0.0 ? ff_.kS * std::copysign(1.0, w_ref) : 0.0)
        + ff_.kV * w_ref
        + ff_.kA * a_ref;

    const double err = w_ref - w_meas;

    const double u_p = g_.kp * err;
    const double u_d = -g_.kd * d_filt_;

    double u_raw = u_ff + u_p + g_.ki * i_ + u_d;

    const double u_cmd = std::clamp(u_raw, -std::abs(u_max), std::abs(u_max));
    if (out_sat) *out_sat = (u_cmd != u_raw);

    if (g_.ki > 0.0 && dt > 0.0)
    {
        const double unclamped_to_cmd_error = u_cmd - u_raw;
        if (u_cmd == u_raw || (unclamped_to_cmd_error * err) > 0.0)
        {
            i_ += (err + k_aw_ * unclamped_to_cmd_error) * dt;
        }
    }

    return u_cmd;
}

void VelocityController::reset()
{
    i_ = 0.0;
    w_meas_prev_ = 0.0;
    d_filt_ = 0.0;
}
