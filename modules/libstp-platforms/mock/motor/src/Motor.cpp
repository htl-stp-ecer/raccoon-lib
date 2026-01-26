#include "hal/Motor.hpp"
#include <stdexcept>
#include "core/MockPlatform.hpp"
#include "foundation/config.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted,
                                 const foundation::MotorCalibration& calibration) : port_(port), inverted_(inverted),
    calibration_(calibration)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    platform::mock::core::MockPlatform::instance().init();
    LIBSTP_LOG_INFO(
        "Mock Motor ctor port={} inverted={} pid(kp={}, ki={}, kd={}) ff(kS={}, kV={}, kA={})",
        port_,
        inverted_,
        calibration_.pid.kp,
        calibration_.pid.ki,
        calibration_.pid.kd,
        calibration_.ff.kS,
        calibration_.ff.kV,
        calibration_.ff.kA);
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
    LIBSTP_LOG_INFO(
        "Mock Motor port={} setSpeed percent={} dir={} duty={} inverted={}",
        port_,
        percent,
        static_cast<int>(dir),
        duty,
        inverted_);
    setMotor(port_, dir, duty);
}

int libstp::hal::motor::Motor::getPosition() const
{
    const int value = static_cast<int>(platform::mock::core::bemf(port_));
    LIBSTP_LOG_TRACE("Mock Motor port={} getPosition -> {}", port_, value);
    return value;
}

void libstp::hal::motor::Motor::brake()
{
    platform::mock::core::setMotor(port_, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor port={} brake", port_);
}

void libstp::hal::motor::Motor::disableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::mock::core::setMotor(p, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor disableAll executed");
}

const libstp::foundation::MotorCalibration& libstp::hal::motor::Motor::getCalibration() const
{
    return calibration_;
}

void libstp::hal::motor::Motor::setCalibration(const foundation::MotorCalibration& calibration)
{
    calibration_ = calibration;
    LIBSTP_LOG_INFO("Mock Motor port={} setCalibration ticks_to_rad={}", port_, calibration_.ticks_to_rad);
}
