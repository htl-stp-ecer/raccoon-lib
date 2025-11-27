//
// Created by tobias on 9/8/25.
//
#include "drive/motor_adapter.hpp"
#include "calibration/calibration.hpp"
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
        LIBSTP_LOG_WARN("MotorAdapter constructed with null motor pointer");
        return;
    }

    LIBSTP_LOG_INFO(
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
        LIBSTP_LOG_INFO("MotorAdapter::updateEncoderVelocity skipped: null motor");
        return;
    }

    if (dt <= 0.0)
    {
        LIBSTP_LOG_INFO("MotorAdapter::updateEncoderVelocity skipped: non-positive dt={}", dt);
        return;
    }

    const long long pos = motor_->getPosition();
    LIBSTP_LOG_TRACE(
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
        LIBSTP_LOG_TRACE("MotorAdapter::updateEncoderVelocity initializing position history for port={}", motor_->port);
        return;
    }

    const long long d_ticks = pos - pos_prev_;

    constexpr long long kMaxDeltaTicks = 10000;
    if (std::abs(d_ticks) > kMaxDeltaTicks)
    {
        LIBSTP_LOG_WARN(
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
    LIBSTP_LOG_TRACE(
        "MotorAdapter::updateEncoderVelocity port={} d_ticks={} raw_w={} filt_w={} alpha={}",
        motor_->port,
        d_ticks,
        w,
        w_meas_filt_,
        a);
}

double MotorAdapter::getVelocity() const
{
    LIBSTP_LOG_TRACE(
        "MotorAdapter::getVelocity port={} returning filt_w={}",
        motor_ ? motor_->port : -1,
        w_meas_filt_);
    return w_meas_filt_;
}

int MotorAdapter::getRawPercent() const
{
    if (!motor_) return 0;
    const int pos = motor_->getPosition();
    LIBSTP_LOG_TRACE("MotorAdapter::getRawPercent port={} bemf={}", motor_->port, pos);
    return pos;
}

void MotorAdapter::setVelocityWithAccel(double w_ref, double a_ref, double dt, bool* out_saturated)
{
    if (!motor_)
    {
        LIBSTP_LOG_INFO("MotorAdapter::setVelocityWithAccel skipped: null motor");
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

    LIBSTP_LOG_INFO(
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
    LIBSTP_LOG_INFO(
        "MotorAdapter::setVelocity port={} w_ref={} dt={}",
        motor_ ? motor_->port : -1,
        w_ref,
        dt);
}

void MotorAdapter::setPercent(double percent)
{
    if (!motor_)
    {
        LIBSTP_LOG_INFO("MotorAdapter::setPercent skipped: null motor");
        return;
    }

    controller_.reset();
    double u = std::clamp(percent, -u_max, u_max);
    motor_->setSpeed(static_cast<int>(std::lround(u)));
    last_u_cmd_ = u;
    LIBSTP_LOG_INFO(
        "MotorAdapter::setPercent port={} percent={} clamped={}",
        motor_->port,
        percent,
        u);
}

void MotorAdapter::resetController()
{
    controller_.reset();
    LIBSTP_LOG_TRACE("MotorAdapter::resetController port={}", motor_ ? motor_->port : -1);
}

void MotorAdapter::brake()
{
    if (!motor_)
    {
        LIBSTP_LOG_INFO("MotorAdapter::brake skipped: null motor");
        return;
    }
    motor_->brake();
    resetController();
    LIBSTP_LOG_INFO("MotorAdapter::brake port={} last_u_cmd={} reset controller", motor_->port, last_u_cmd_);
}

void MotorAdapter::resetEncoderTracking()
{
    pos_prev_init_ = false;
    w_meas_filt_ = 0.0;
    LIBSTP_LOG_INFO("MotorAdapter::resetEncoderTracking port={} - cleared position history", motor_ ? motor_->port : -1);
}


libstp::hal::motor::Motor& MotorAdapter::motor() { return *motor_; }
const libstp::hal::motor::Motor& MotorAdapter::motor() const { return *motor_; }

CalibrationResult MotorAdapter::calibrate()
{
    return calibrate(CalibrationConfig{});
}

CalibrationResult MotorAdapter::calibrate(const CalibrationConfig& config)
{
    if (!motor_)
    {
        CalibrationResult result;
        result.success = false;
        result.error_message = "Cannot calibrate: null motor pointer";
        LIBSTP_LOG_ERROR("{}", result.error_message);
        return result;
    }

    LIBSTP_LOG_INFO("Starting calibration for motor on port {}", motor_->port);

    // Create calibrator and run calibration
    MotorCalibrator calibrator(*motor_, config);
    CalibrationResult result = calibrator.calibrate();

    if (result.success)
    {
        // Apply the calibrated parameters
        foundation::MotorCalibration new_cal;
        new_cal.pid = result.pid;
        new_cal.ff = result.ff;
        // Preserve other calibration parameters
        new_cal.ticks_to_rad = motor_->getCalibration().ticks_to_rad;
        new_cal.vel_lpf_alpha = motor_->getCalibration().vel_lpf_alpha;

        updateCalibration(new_cal);
        LIBSTP_LOG_INFO("Calibration completed successfully for motor on port {}", motor_->port);
    }
    else
    {
        LIBSTP_LOG_ERROR("Calibration failed for motor on port {}: {}",
                    motor_->port, result.error_message);
    }

    return result;
}

void MotorAdapter::updateCalibration(const foundation::MotorCalibration& cal)
{
    if (!motor_)
    {
        LIBSTP_LOG_WARN("Cannot update calibration: null motor pointer");
        return;
    }

    // Update the controller gains
    controller_.setGains(cal.pid);
    controller_.setFF(cal.ff);

    LIBSTP_LOG_INFO("Updated calibration for motor on port {}: kp={:.3f}, ki={:.3f}, kd={:.3f}, kS={:.3f}, kV={:.3f}, kA={:.3f}",
                motor_->port,
                cal.pid.kp, cal.pid.ki, cal.pid.kd,
                cal.ff.kS, cal.ff.kV, cal.ff.kA);
}
