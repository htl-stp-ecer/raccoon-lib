//
// Created by tobias on 6/9/25.
//
#include "hal/Motor.hpp"
#include "foundation/config.hpp"

using namespace libstp::hal::motor;

#ifdef SAFETY_CHECKS_ENABLED
void Motor::registerMotorPort(int port)
{
    SPDLOG_DEBUG("Registering motor port {}", port);
    if (used_motor_ports.contains(port))
    {
        SPDLOG_WARN("Motor port {} is already in use!", port);
        return;
    }

    used_motor_ports.insert(port);
}

void Motor::unregisterMotorPort(const int port)
{
    SPDLOG_DEBUG("Unregistering motor port {}", port);
    used_motor_ports.erase(port);
}
#endif


Motor::~Motor()
{
    SPDLOG_DEBUG("Destroying motor on port {}", port);
    brake();
#ifdef SAFETY_CHECKS_ENABLED
    unregisterMotorPort(port);
#endif
}
