//
// Created by tobias on 9/8/25.
//
#include "drive/motor_adapter.hpp"
#include "foundation/config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace libstp::drive;
constexpr double u_max = 100.0;

MotorAdapter::MotorAdapter(hal::motor::Motor* motor)
    : motor_(motor),
      controller_(motor->getCalibration().pid, motor->getCalibration().ff)
{
    if (!motor_)
    {
        SPDLOG_WARN("MotorAdapter constructed with null motor pointer");
        return;
    }

    SPDLOG_INFO(
        "MotorAdapter::ctor port={} inverted={} pid(kp={}, ki={}, kd={}) ff(kS={}, kV={}, kA={})",
        motor_->port,
        motor_->inverted,
        controller_.gains().kp,
        controller_.gains().ki,
        controller_.gains().kd,
        controller_.ff().kS,
        controller_.ff().kV,
        controller_.ff().kA);
}

void MotorAdapter::updateEncoderVelocity(double dt)
{
    if (!motor_)
    {
        SPDLOG_INFO("MotorAdapter::updateEncoderVelocity skipped: null motor");
        return;
    }

    if (dt <= 0.0)
    {
        SPDLOG_INFO("MotorAdapter::updateEncoderVelocity skipped: non-positive dt={}", dt);
        return;
    }

    const long long pos = motor_->getPosition();
    SPDLOG_TRACE(
        "MotorAdapter::updateEncoderVelocity port={} dt={} pos={} pos_prev={} initialized={}",
        motor_->port,
        dt,
        pos,
        pos_prev_,
        pos_prev_init_);
    if (!pos_prev_init_)
    {
        pos_prev_ = pos;
        pos_prev_init_ = true;
        SPDLOG_TRACE("MotorAdapter::updateEncoderVelocity initializing position history for port={}", motor_->port);
        return;
    }

    const long long d_ticks = pos - pos_prev_;

    constexpr long long kMaxDeltaTicks = 10000;
    if (std::abs(d_ticks) > kMaxDeltaTicks)
    {
        SPDLOG_WARN(
            "MotorAdapter::updateEncoderVelocity port={} detected implausible delta {} ticks (prev={}, cur={}) – reinitializing baseline",
            motor_->port,
            d_ticks,
            pos_prev_,
            pos);
        pos_prev_ = pos;
        w_meas_filt_ = 0.0;
        return;
    }

    pos_prev_ = pos;

    double w = (static_cast<double>(d_ticks) * motor_->getCalibration().ticks_to_rad) / dt; // rad/s

    const double a = std::clamp(motor_->getCalibration().vel_lpf_alpha, 0.0, 1.0);
    w_meas_filt_ = (1.0 - a) * w_meas_filt_ + a * w;
    SPDLOG_TRACE(
        "MotorAdapter::updateEncoderVelocity port={} d_ticks={} raw_w={} filt_w={} alpha={}",
        motor_->port,
        d_ticks,
        w,
        w_meas_filt_,
        a);
}

double MotorAdapter::getVelocity() const
{
    SPDLOG_TRACE(
        "MotorAdapter::getVelocity port={} returning filt_w={}",
        motor_ ? motor_->port : -1,
        w_meas_filt_);
    return w_meas_filt_;
}

int MotorAdapter::getRawPercent() const
{
    if (!motor_) return 0;
    const int pos = motor_->getPosition();
    SPDLOG_TRACE("MotorAdapter::getRawPercent port={} bemf={}", motor_->port, pos);
    return pos;
}

void MotorAdapter::setVelocityWithAccel(double w_ref, double a_ref, double dt, bool* out_saturated)
{
    if (!motor_)
    {
        SPDLOG_INFO("MotorAdapter::setVelocityWithAccel skipped: null motor");
        if (out_saturated) *out_saturated = false;
        return;
    }

    updateEncoderVelocity(dt);
    double w_meas = getVelocity();

    bool saturated = false;
    double u = controller_.compute(w_ref, a_ref, w_meas, dt, u_max, &saturated);

    u = std::clamp(u, -u_max, u_max);

    motor_->setSpeed(static_cast<int>(std::lround(u)));
    last_u_cmd_ = u;
    if (out_saturated) *out_saturated = saturated || (std::abs(u) >= u_max - 1e-6);

    SPDLOG_INFO(
        "MotorAdapter::setVelocityWithAccel port={} w_ref={} a_ref={} dt={} w_meas={} u_cmd={} saturated={} limited_cmd={}",
        motor_->port,
        w_ref,
        a_ref,
        dt,
        w_meas,
        u,
        saturated,
        out_saturated ? *out_saturated : saturated);
}

void MotorAdapter::setVelocity(double w_ref, double dt)
{
    bool dummy = false;
    setVelocityWithAccel(w_ref, 0.0, dt, &dummy);
    SPDLOG_INFO(
        "MotorAdapter::setVelocity port={} w_ref={} dt={}",
        motor_ ? motor_->port : -1,
        w_ref,
        dt);
}

void MotorAdapter::setPercent(double percent)
{
    if (!motor_)
    {
        SPDLOG_INFO("MotorAdapter::setPercent skipped: null motor");
        return;
    }

    controller_.reset();
    double u = std::clamp(percent, -u_max, u_max);
    motor_->setSpeed(static_cast<int>(std::lround(u)));
    last_u_cmd_ = u;
    SPDLOG_INFO(
        "MotorAdapter::setPercent port={} percent={} clamped={}",
        motor_->port,
        percent,
        u);
}

void MotorAdapter::resetController()
{
    controller_.reset();
    SPDLOG_TRACE("MotorAdapter::resetController port={}", motor_ ? motor_->port : -1);
}

void MotorAdapter::brake()
{
    if (!motor_)
    {
        SPDLOG_INFO("MotorAdapter::brake skipped: null motor");
        return;
    }
    motor_->brake();
    resetController();
    SPDLOG_INFO("MotorAdapter::brake port={} last_u_cmd={} reset controller", motor_->port, last_u_cmd_);
}


libstp::hal::motor::Motor& MotorAdapter::motor() { return *motor_; }
const libstp::hal::motor::Motor& MotorAdapter::motor() const { return *motor_; }
