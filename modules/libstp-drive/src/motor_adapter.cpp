//
// Created by tobias on 9/8/25.
//
#include "drive/motor_adapter.hpp"
#include <algorithm>
#include <cmath>

using namespace libstp::drive;

MotorAdapter::MotorAdapter(hal::motor::Motor* motor)
    : motor_(motor),
      controller_(motor->getCalibration().pid, motor->getCalibration().ff, motor->getCalibration().deadzone)
{
}

void MotorAdapter::updateEncoderVelocity(double dt)
{
    if (!motor_ || dt <= 0.0) return;

    const long long pos = motor_->getPosition();
    if (!pos_prev_init_)
    {
        pos_prev_ = pos;
        pos_prev_init_ = true;
        return;
    }

    const long long d_ticks = pos - pos_prev_;
    pos_prev_ = pos;

    double w = (static_cast<double>(d_ticks) * motor_->getCalibration().ticks_to_rad) / dt; // rad/s
    //if (motor_->getCalibration().invert_meas) w = -w;

    const double a = std::clamp(motor_->getCalibration().vel_lpf_alpha, 0.0, 1.0);
    w_meas_filt_ = (1.0 - a) * w_meas_filt_ + a * w;
}

double MotorAdapter::getVelocity() const
{
    return w_meas_filt_;
}

int MotorAdapter::getRawPercent() const
{
    if (!motor_) return 0;
    return motor_->getPosition();
}

void MotorAdapter::setVelocityWithAccel(double w_ref, double a_ref, double dt, bool* out_saturated)
{
    if (!motor_) return;

    updateEncoderVelocity(dt);
    double w_meas = getVelocity();

    const double u_max = std::abs(motor_->getCalibration().max_percent_output);
    bool saturated = false;
    double u = controller_.compute(w_ref, a_ref, w_meas, dt, u_max, &saturated);

    //if (motor_->getCalibration().invert_cmd) u = -u;

    u = std::clamp(u, -u_max, u_max);

    motor_->setSpeed(static_cast<int>(std::lround(u)));
    last_u_cmd_ = u;
    if (out_saturated) *out_saturated = saturated || (std::abs(u) >= u_max - 1e-6);
}

void MotorAdapter::setVelocity(double w_ref, double dt)
{
    bool dummy = false;
    setVelocityWithAccel(w_ref, 0.0, dt, &dummy);
}

void MotorAdapter::setPercent(double percent)
{
    if (!motor_) return;

    controller_.reset();
    const double u_max = std::abs(motor_->getCalibration().max_percent_output);
    double u = std::clamp(percent, -u_max, u_max);
    //if (motor_->getCalibration().invert_cmd) u = -u;
    motor_->setSpeed(static_cast<int>(std::lround(u)));
    last_u_cmd_ = u;
}

void MotorAdapter::resetController()
{
    controller_.reset();
}

void MotorAdapter::brake()
{
    if (!motor_) return;
    motor_->brake();
    resetController();
}


libstp::hal::motor::Motor& MotorAdapter::motor() { return *motor_; }
const libstp::hal::motor::Motor& MotorAdapter::motor() const { return *motor_; }
