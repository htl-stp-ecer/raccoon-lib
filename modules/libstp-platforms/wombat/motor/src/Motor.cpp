//
// Created by tobias on 6/1/25.
//

#include "hal/Motor.hpp"

#include <stdexcept>

#include "core/Spi.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted, const foundation::MotorCalibration& calibration): port(port), inverted(inverted), calibration_(calibration)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    platform::wombat::core::Spi::instance().init();
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

    using namespace platform::wombat::core;
    const bool zero = percent == 0;
    auto dir = MotorDir::Off;
    if (!zero)
        dir = (inverted ? percent < 0 : percent > 0) ? MotorDir::CW : MotorDir::CCW;
    const uint32_t duty = static_cast<uint32_t>(percent * 4); // 0-400
    setMotor(port, dir, duty);
}

int libstp::hal::motor::Motor::getPosition() const
{
    return static_cast<int>(platform::wombat::core::bemf(port));
}

void libstp::hal::motor::Motor::brake() const
{
    platform::wombat::core::setMotor(port, platform::wombat::core::MotorDir::Off, 0);
}

void libstp::hal::motor::Motor::disableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::wombat::core::setMotor(p, platform::wombat::core::MotorDir::Off, 0);
}

const libstp::foundation::MotorCalibration& libstp::hal::motor::Motor::getCalibration() const
{
    return calibration_;
}