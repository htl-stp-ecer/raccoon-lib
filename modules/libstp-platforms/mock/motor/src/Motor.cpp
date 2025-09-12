#include "hal/Motor.hpp"
#include <stdexcept>
#include "core/MockPlatform.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted): port(port), inverted(inverted)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    platform::mock::core::MockPlatform::instance().init();
}

libstp::hal::motor::Motor::~Motor()
{
#ifdef SAFETY_CHECKS_ENABLED
    unregisterMotorPort(port);
#endif
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
    const uint32_t duty = static_cast<uint32_t>(std::abs(percent) * 4); // 0-400
    setMotor(port, dir, duty);
}

int libstp::hal::motor::Motor::getPosition() const
{
    return static_cast<int>(platform::mock::core::bemf(port));
}

void libstp::hal::motor::Motor::brake() const
{
    platform::mock::core::setMotor(port, platform::mock::core::MotorDir::Off, 0);
}

void libstp::hal::motor::Motor::disableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::mock::core::setMotor(p, platform::mock::core::MotorDir::Off, 0);
}