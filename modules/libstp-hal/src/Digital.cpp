#include "hal/Digital.hpp"
#include "foundation/config.hpp"

#ifdef SAFETY_CHECKS_ENABLED
void libstp::hal::digital::DigitalSensor::registerDigitalPort(int port)
{
    if (!registry_.tryRegister(port))
    {
        LIBSTP_LOG_WARN("Digital port {} is already in use!", port);
    }
}

void libstp::hal::digital::DigitalSensor::unregisterDigitalPort(const int port)
{
    registry_.unregister(port);
}
#endif
