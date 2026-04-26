#include "hal/Digital.hpp"

#include <stdexcept>

#include "core/LcmReader.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 10;

libstp::hal::digital::DigitalSensor::DigitalSensor(const int port): port(port)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerDigitalPort(port);
#endif
}

libstp::hal::digital::DigitalSensor::~DigitalSensor()
{
#ifdef SAFETY_CHECKS_ENABLED
    unregisterDigitalPort(port);
#endif
}

bool libstp::hal::digital::DigitalSensor::read() const
{
    // readDigital(port) returns 0 or 1 for the specific port, not a bitmask
    const int digital = platform::wombat::core::LcmReader::instance().readDigital(port).value;
    return digital != 0;
}
