#include "drive/motor_adapter.hpp"
#include "foundation/config.hpp"
#include "foundation/speed_mode_context.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace libstp::drive;
// BEMF sampling rate in Hz (5ms interval on firmware)
constexpr double kBemfSampleRate = 200.0;

MotorAdapter::MotorAdapter(hal::motor::IMotor* motor)
    : motor_(motor)
{
    if (!motor_)
    {
        LIBSTP_LOG_WARN("MotorAdapter constructed with null motor pointer");
        return;
    }

    LIBSTP_LOG_TRACE(
        "MotorAdapter::ctor port={} inverted={}",
        motor_->getPort(),
        motor_->isInverted());
}

void MotorAdapter::updateEncoderVelocity(double dt)
{
    if (!motor_) return;
    if (dt <= 0.0) return;

    // Position is the accumulated BEMF-tick counter on the STM32 (per the
    // firmware contract: position[] keeps integrating in every motor mode).
    // We differentiate it here and smooth with the per-motor IIR alpha.
    const long long pos = motor_->getPosition();
    if (!pos_prev_init_)
    {
        pos_prev_ = pos;
        pos_prev_init_ = true;
        return;
    }

    const long long d_ticks = pos - pos_prev_;

    // Plausibility bound: more than 5 wheel revolutions in one update is
    // almost certainly a counter wrap, a reset between calls, or a stale
    // pre-reset sample. Resync without distorting the filter.
    const double ticks_to_rad = motor_->getCalibration().ticks_to_rad;
    const long long kMaxDeltaTicks = (ticks_to_rad > 0.0)
        ? static_cast<long long>(5.0 * 2.0 * M_PI / ticks_to_rad)
        : 500000;
    if (std::abs(d_ticks) > kMaxDeltaTicks)
    {
        LIBSTP_LOG_WARN(
            "MotorAdapter::updateEncoderVelocity port={} implausible delta {} ticks "
            "(prev={}, cur={}) — resyncing baseline",
            motor_->getPort(), d_ticks, pos_prev_, pos);
        pos_prev_ = pos;
        w_meas_filt_ = 0.0;
        return;
    }

    pos_prev_ = pos;

    const double w = (static_cast<double>(d_ticks) * ticks_to_rad) / dt; // rad/s
    const double a = std::clamp(motor_->getCalibration().vel_lpf_alpha, 0.0, 1.0);
    w_meas_filt_ = (1.0 - a) * w_meas_filt_ + a * w;

    LIBSTP_LOG_TRACE(
        "MotorAdapter::updateEncoderVelocity port={} d_ticks={} dt={} raw_w={} filt_w={} alpha={}",
        motor_->getPort(), d_ticks, dt, w, w_meas_filt_, a);
}

double MotorAdapter::getVelocity() const
{
    return w_meas_filt_;
}

void MotorAdapter::setVelocity(double w_ref, double dt, bool* out_saturated)
{
    if (!motor_)
    {
        if (out_saturated) *out_saturated = false;
        return;
    }

    // Convert rad/s to BEMF units and send to firmware (PID runs on STM32)
    const double ticks_to_rad = motor_->getCalibration().ticks_to_rad;
    const int bemf_target = (ticks_to_rad > 0.0)
        ? static_cast<int>(std::lround(w_ref / (ticks_to_rad * kBemfSampleRate)))
        : 0;

    motor_->setVelocity(bemf_target);
    if (out_saturated) *out_saturated = false;

    // Encoder velocity tracking is meaningless when BEMF is off — positions
    // are frozen on the firmware side. Skip the update to avoid noise spikes
    // when SpeedMode is toggled mid-run.
    if (!libstp::foundation::SpeedModeContext::instance().isSpeedModeEnabled())
    {
        updateEncoderVelocity(dt);
    }

    LIBSTP_LOG_TRACE(
        "MotorAdapter::setVelocity port={} w_ref={} bemf_target={} dt={}",
        motor_->getPort(), w_ref, bemf_target, dt);
}

void MotorAdapter::brake()
{
    if (!motor_) return;
    motor_->brake();
    LIBSTP_LOG_TRACE("MotorAdapter::brake port={}", motor_->getPort());
}

void MotorAdapter::resetEncoderTracking()
{
    pos_prev_init_ = false;
    w_meas_filt_ = 0.0;
    LIBSTP_LOG_TRACE("MotorAdapter::resetEncoderTracking port={}",
                     motor_ ? motor_->getPort() : -1);
}


libstp::hal::motor::IMotor& MotorAdapter::motor() { return *motor_; }
const libstp::hal::motor::IMotor& MotorAdapter::motor() const { return *motor_; }
