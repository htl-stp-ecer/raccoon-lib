#include "hal/Motor.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include "core/MockPlatform.hpp"
#include "foundation/config.hpp"
#include "foundation/speed_mode_context.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted,
                                 const foundation::MotorCalibration& calibration) : port_(port), inverted_(inverted),
    calibration_(calibration)
{
    calibration_.validate();
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    platform::mock::core::MockPlatform::instance().init();
    LIBSTP_LOG_INFO(
        "Mock Motor ctor port={} inverted={}",
        port_,
        inverted_);
    if (calibration_.pid)
    {
        setFirmwarePidGains(static_cast<float>(calibration_.pid->kp),
                            static_cast<float>(calibration_.pid->ki),
                            static_cast<float>(calibration_.pid->kd));
    }
}

void libstp::hal::motor::Motor::setSpeed(const int percent)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (percent < MIN_SPEED || percent > MAX_SPEED) throw std::out_of_range("speed -100 - 100");
#endif

    using namespace platform::mock::core;
    const bool zero = percent == 0;
    auto dir = MotorDir::Off;
    if (!zero)
        dir = (inverted_ ? percent < 0 : percent > 0) ? MotorDir::CW : MotorDir::CCW;
    const auto duty = static_cast<uint32_t>(std::abs(percent) * 4); // 0-400
    LIBSTP_LOG_DEBUG(
        "Mock Motor port={} setSpeed percent={} dir={} duty={} inverted={}",
        port_,
        percent,
        static_cast<int>(dir),
        duty,
        inverted_);
    setMotor(port_, dir, duty);
}

void libstp::hal::motor::Motor::setVelocity(const int velocity)
{
    const int directionVelocity = inverted_ ? -velocity : velocity;
    LIBSTP_LOG_DEBUG("Mock Motor port={} setVelocity={} inverted={}", port_, velocity, inverted_);
    // Closed-loop velocity command — uses the BEMF-units path so the sim can
    // interpret it via the motor calibration instead of as a duty %.
    platform::mock::core::MockPlatform::instance().setMotorVelocity(port_, directionVelocity);

    if (sim_active_)
    {
        // Latch the simulated step target sign from the most recent command.
        // We assume the caller provided a non-zero steady-state via
        // setSimulatedBemfResponse() and just restart the response clock here.
        sim_t_ref_ = std::chrono::steady_clock::now();
    }
}

void libstp::hal::motor::Motor::moveToPosition(const int velocity, const int goalPosition)
{
    LIBSTP_LOG_DEBUG("Mock Motor port={} moveToPosition vel={} goal={}", port_, velocity, goalPosition);
    // Mock: store goal, simulate immediate completion
}

void libstp::hal::motor::Motor::moveRelative(const int velocity, const int deltaPosition)
{
    LIBSTP_LOG_DEBUG("Mock Motor port={} moveRelative vel={} delta={}", port_, velocity, deltaPosition);
    // Mock: store delta, simulate immediate completion
}

int libstp::hal::motor::Motor::getPosition() const
{
    foundation::SpeedModeContext::instance().assertBemfAvailable("Motor::getPosition");
    const int value = platform::mock::core::MockPlatform::instance().getMotorPosition(port_);
    LIBSTP_LOG_TRACE("Mock Motor port={} getPosition -> {}", port_, value);
    return value;
}

int libstp::hal::motor::Motor::getBemf() const
{
    if (sim_active_)
    {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - sim_t_ref_).count();
        const double tau = (sim_time_constant_s_ > 1e-6) ? sim_time_constant_s_ : 1e-6;
        const double resp = static_cast<double>(sim_steady_state_) *
                            (1.0 - std::exp(-t / tau));
        const int value = static_cast<int>(std::lround(resp));
        LIBSTP_LOG_TRACE("Mock Motor port={} getBemf (sim t={:.3f}s) -> {}",
                         port_, t, value);
        return value;
    }
    const int value = static_cast<int>(platform::mock::core::bemf(port_));
    LIBSTP_LOG_TRACE("Mock Motor port={} getBemf -> {}", port_, value);
    return value;
}

bool libstp::hal::motor::Motor::isDone() const
{
    LIBSTP_LOG_TRACE("Mock Motor port={} isDone -> true", port_);
    return true;
}

void libstp::hal::motor::Motor::brake()
{
    platform::mock::core::setMotor(port_, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor port={} brake", port_);
}

void libstp::hal::motor::Motor::off()
{
    platform::mock::core::setMotor(port_, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor port={} off", port_);
}

void libstp::hal::motor::Motor::resetPositionCounter()
{
    LIBSTP_LOG_DEBUG("Mock Motor port={} resetPositionCounter", port_);
    // Mock: no-op, position comes from bemf() which is simulated
}

void libstp::hal::motor::Motor::disableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::mock::core::setMotor(p, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor disableAll executed");
}

void libstp::hal::motor::Motor::enableAll()
{
    LIBSTP_LOG_INFO("Mock Motor enableAll executed (no-op on mock)");
}

const libstp::foundation::MotorCalibration& libstp::hal::motor::Motor::getCalibration() const
{
    return calibration_;
}

void libstp::hal::motor::Motor::setCalibration(const foundation::MotorCalibration& calibration)
{
    calibration.validate();
    calibration_ = calibration;
    if (calibration_.pid)
    {
        setFirmwarePidGains(static_cast<float>(calibration_.pid->kp),
                            static_cast<float>(calibration_.pid->ki),
                            static_cast<float>(calibration_.pid->kd));
    }
    LIBSTP_LOG_INFO("Mock Motor port={} setCalibration ticks_to_rad={}", port_, calibration_.ticks_to_rad);
}

void libstp::hal::motor::Motor::setFirmwarePidGains(float kp, float ki, float kd)
{
    last_fw_kp_ = kp;
    last_fw_ki_ = ki;
    last_fw_kd_ = kd;
    LIBSTP_LOG_INFO("Mock Motor port={} setFirmwarePidGains kp={} ki={} kd={}",
                    port_, kp, ki, kd);
}

void libstp::hal::motor::Motor::setSimulatedBemfResponse(int steady_state_bemf,
                                                         double time_constant_s)
{
    sim_active_          = true;
    sim_steady_state_    = steady_state_bemf;
    sim_time_constant_s_ = (time_constant_s > 1e-6) ? time_constant_s : 1e-6;
    sim_t_ref_           = std::chrono::steady_clock::now();
    LIBSTP_LOG_INFO("Mock Motor port={} setSimulatedBemfResponse ss={} tau={}s",
                    port_, steady_state_bemf, sim_time_constant_s_);
}

void libstp::hal::motor::Motor::clearSimulatedBemfResponse()
{
    sim_active_ = false;
    LIBSTP_LOG_INFO("Mock Motor port={} clearSimulatedBemfResponse", port_);
}

void libstp::hal::motor::Motor::getLastFirmwarePidGains(float& kp, float& ki, float& kd) const
{
    kp = last_fw_kp_;
    ki = last_fw_ki_;
    kd = last_fw_kd_;
}
