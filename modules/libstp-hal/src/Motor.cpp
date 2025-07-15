//
// Created by tobias on 6/9/25.
//
#include "hal/Motor.hpp"
#include "foundation/config.hpp"

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::motor::Motor::registerMotorPort(int port)
{
    if (used_motor_ports.contains(port))
    {
        SPDLOG_WARN("Motor port {} is already in use!", port);
        return;
    }

    used_motor_ports.insert(port);
}

void libstp::hal::motor::Motor::unregisterMotorPort(const int port)
{
    used_motor_ports.erase(port);
}
#endif
