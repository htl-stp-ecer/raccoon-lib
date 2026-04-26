//
// Created by tobias on 6/9/25.
//

#include "hal/Servo.hpp"
#include "foundation/config.hpp"

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::servo::Servo::registerServoPort(int port)
{
    if (!registry_.tryRegister(port))
    {
        LIBSTP_LOG_WARN("Servo port {} is already in use!", port);
    }
}

void libstp::hal::servo::Servo::unregisterServoPort(const int port)
{
    registry_.unregister(port);
}
#endif
