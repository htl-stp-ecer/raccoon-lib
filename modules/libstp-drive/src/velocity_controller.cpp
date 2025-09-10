//
// Created by tobias on 9/8/25.
//
#include "drive/velocity_controller.hpp"

libstp::drive::VelocityController::VelocityController(Gains g): g_(g)
{
}

void libstp::drive::VelocityController::setGains(const Gains& g)
{
    g_ = g;
}

const libstp::drive::VelocityController::Gains& libstp::drive::VelocityController::gains() const
{
    return g_;
}

double libstp::drive::VelocityController::compute(const double target_w, const double meas_w, const double dt)
{
    const double err = target_w - meas_w;
    i_ += err * dt;
    const double d = dt > 0.0 ? (err - pe_) / dt : 0.0;
    pe_ = err;
    return g_.ff * target_w + g_.kp * err + g_.ki * i_ + g_.kd * d;
}

void libstp::drive::VelocityController::reset()
{
    i_ = 0.0;
    pe_ = 0.0;
}
