//
// Created by tobias on 6/9/25.
//
#include "hal/Digital.hpp"
#include "foundation/config.hpp"

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::digital::DigitalSensor::registerDigitalPort(int port)
{
    if (used_digital_ports.contains(port))
    {
        LIBSTP_LOG_WARN("Digital port {} is already in use!", port);
        return;
    }

    used_digital_ports.insert(port);
}
void libstp::hal::digital::DigitalSensor::unregisterDigitalPort(const int port)
{
    used_digital_ports.erase(port);
}
#endif