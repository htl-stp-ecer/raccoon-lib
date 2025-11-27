#include "hal/Motor.hpp"
#include <stdexcept>
#include "core/MockPlatform.hpp"
#include "foundation/config.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted,
                                 const foundation::MotorCalibration& calibration) : port(port), inverted(inverted),
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
        this->port,
        this->inverted,
        calibration_.pid.kp,
        calibration_.pid.ki,
        calibration_.pid.kd,
        calibration_.ff.kS,
        calibration_.ff.kV,
        calibration_.ff.kA);
}

void libstp::hal::motor::Motor::setSpeed(const int percent) const
{
#ifdef SAFETY_CHECKS_ENABLED
    if (percent < MIN_SPEED || percent > MAX_SPEED) throw std::out_of_range("speed -100 - 100");
#endif

    using namespace platform::mock::core;
    const bool zero = percent == 0;
    auto dir = MotorDir::Off;
    if (!zero)
        dir = (inverted ? percent < 0 : percent > 0) ? MotorDir::CW : MotorDir::CCW;
    const auto duty = static_cast<uint32_t>(std::abs(percent) * 4); // 0-400
    LIBSTP_LOG_INFO(
        "Mock Motor port={} setSpeed percent={} dir={} duty={} inverted={}",
        port,
        percent,
        static_cast<int>(dir),
        duty,
        inverted);
    setMotor(port, dir, duty);
}

int libstp::hal::motor::Motor::getPosition() const
{
    const int value = static_cast<int>(platform::mock::core::bemf(port));
    LIBSTP_LOG_TRACE("Mock Motor port={} getPosition -> {}", port, value);
    return value;
}

void libstp::hal::motor::Motor::brake() const
{
    platform::mock::core::setMotor(port, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor port={} brake", port);
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
