#include "hal/Analog.hpp"
#include "foundation/config.hpp"

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::analog::AnalogSensor::registerAnalogPort(int port)
{
    if (!registry_.tryRegister(port))
    {
        LIBSTP_LOG_WARN("Analog port {} is already in use!", port);
    }
}

void libstp::hal::analog::AnalogSensor::unregisterAnalogPort(int port)
{
    registry_.unregister(port);
}
#endif
