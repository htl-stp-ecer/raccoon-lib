//
// Created by tobias on 6/9/25.
//

#include "hal/Servo.hpp"
#include "foundation/config.hpp"

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::servo::Servo::registerServoPort(int port)
{
    if (used_servo_ports.contains(port))
    {
        LIBSTP_LOG_WARN("Servo port {} is already in use!", port);
        return;
    }

    used_servo_ports.insert(port);
}

void libstp::hal::servo::Servo::unregisterServoPort(const int port)
{
    used_servo_ports.erase(port);
}
#endif
