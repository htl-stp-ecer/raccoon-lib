//
// Created by tobias on 6/9/25.
//
#include "hal/Analog.hpp"
#include "foundation/config.hpp"
#include <spdlog/spdlog.h>

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 5;

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::analog::AnalogSensor::registerAnalogPort(int port)
{
    if (used_analog_ports.contains(port))
    {
        SPDLOG_WARN("Analog port {} is already in use!", port);
        return;
    }

    used_analog_ports.insert(port);
}

void libstp::hal::analog::AnalogSensor::unregisterAnalogPort(int port)
{
    used_analog_ports.erase(port);
}
#endif