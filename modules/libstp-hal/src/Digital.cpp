#include "hal/Digital.hpp"
#include "foundation/config.hpp"

#include <stdexcept>
#include <utility>

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

namespace libstp::hal::digital
{
    namespace
    {
        int firstPortOrThrow(const std::vector<int>& ports)
        {
            if (ports.empty())
            {
                throw std::invalid_argument("ButtonGroup requires at least one port");
            }
            return ports.front();
        }
    } // namespace

    // The base subobject owns ports_[0]; the rest become extra_ sensors so
    // every member port is registered exactly once.
    ButtonGroup::ButtonGroup(std::vector<int> ports)
        : DigitalSensor(firstPortOrThrow(ports)), ports_(std::move(ports))
    {
        for (std::size_t i = 1; i < ports_.size(); ++i)
        {
            extra_.push_back(std::make_unique<DigitalSensor>(ports_[i]));
        }
    }

    bool ButtonGroup::read() const
    {
        if (DigitalSensor::read())
        {
            return true;
        }
        for (const auto& sensor : extra_)
        {
            if (sensor->read())
            {
                return true;
            }
        }
        return false;
    }
} // namespace libstp::hal::digital
