#include "hal/Servo.hpp"
#include "core/MockPlatform.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;

libstp::hal::servo::Servo::Servo(const int port): port(port)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerServoPort(port);
#endif

    platform::mock::core::MockPlatform::instance().init();
}

libstp::hal::servo::Servo::~Servo()
{
#ifdef SAFETY_CHECKS_ENABLED
    unregisterServoPort(port);
#endif
}

void libstp::hal::servo::Servo::setPosition(const float position)
{
    storedPosition = position;
    platform::mock::core::setServo(port,
                                   platform::mock::core::ServoMode::Enabled,
                                   static_cast<uint16_t>(position));
}

float libstp::hal::servo::Servo::getPosition() const { return storedPosition; }

void libstp::hal::servo::Servo::enable() const
{
    platform::mock::core::setServo(port, platform::mock::core::ServoMode::Enabled, storedPosition);
}

void libstp::hal::servo::Servo::disable() const
{
    platform::mock::core::setServo(port, platform::mock::core::ServoMode::Disabled, storedPosition);
}

void libstp::hal::servo::Servo::fullyDisableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::mock::core::setServo(p, platform::mock::core::ServoMode::FullyDisabled, 0);
}
