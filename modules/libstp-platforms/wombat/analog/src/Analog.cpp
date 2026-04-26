#include "hal/Analog.hpp"

#include <stdexcept>

#include "core/LcmReader.hpp"
constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 5;

libstp::hal::analog::AnalogSensor::AnalogSensor(const int port): port(port)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerAnalogPort(port);
#endif
}


libstp::hal::analog::AnalogSensor::~AnalogSensor()
{
#ifdef SAFETY_CHECKS_ENABLED
    unregisterAnalogPort(port);
#endif
}

int libstp::hal::analog::AnalogSensor::read() const
{
    return platform::wombat::core::LcmReader::instance().readAnalog(port).value;
}
