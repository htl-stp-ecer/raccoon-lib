//
// Created by tobias on 9/8/25.
//
#include "drive/velocity_controller.hpp"
#include <cmath>
#include <algorithm>

using namespace libstp::drive;

VelocityController::VelocityController(PidGains g, Feedforward ff, Deadzone dz)
    : g_(g), ff_(ff), dz_(dz)
{
}

void VelocityController::setGains(const PidGains& g) { g_ = g; }
void VelocityController::setFF(const Feedforward& ff) { ff_ = ff; }
void VelocityController::setDeadzone(const Deadzone& dz) { dz_ = dz; }

double VelocityController::mapDutyWithDeadzone(double u_raw, double u_max) const
{
    if (!dz_.enable) return u_raw;

    const double zw = std::clamp(dz_.zero_window_percent, 0.0, u_max);
    const double sp = std::clamp(dz_.start_percent, 0.0, u_max);
    const double rp = (dz_.release_percent > 0.0)
                          ? std::clamp(dz_.release_percent, 0.0, sp)
                          : zw;

    const double sign = (u_raw >= 0.0) ? 1.0 : -1.0;
    double mag = std::abs(u_raw);

    if (mag <= rp) return 0.0;

    if (mag <= zw) mag = zw;
    const double denom = std::max(1e-6, u_max - zw);
    const double alpha = (mag - zw) / denom; // 0..1
    const double out_mag = sp + alpha * (u_max - sp);

    return std::clamp(sign * out_mag, -u_max, u_max);
}

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

    double u_mapped = mapDutyWithDeadzone(u_raw, std::abs(u_max));

    const double u_cmd = std::clamp(u_mapped, -std::abs(u_max), std::abs(u_max));
    if (out_sat) *out_sat = (u_cmd != u_mapped);

    if (g_.ki > 0.0 && dt > 0.0)
    {
        const double unclamped_to_cmd_error = u_cmd - u_mapped;
        if (u_cmd == u_mapped || (unclamped_to_cmd_error * err) > 0.0)
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
