#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <mutex>
#include <set>
#endif

namespace libstp::hal::detail
{
#ifdef SAFETY_CHECKS_ENABLED
    /**
     * Thread-safe duplicate-port detector for HAL safety checks.
     *
     * Each HAL class that wants duplicate-port detection (Motor, Servo,
     * AnalogSensor, DigitalSensor) holds one static instance and routes
     * its register/unregister calls through it. The mutex serialises
     * concurrent construction across threads — without it, simultaneous
     * `Motor` ctors race on the underlying `std::set` and silently corrupt it.
     */
    class PortRegistry
    {
    public:
        /// Returns true if the port was newly registered, false if it was already in use.
        bool tryRegister(int port)
        {
            std::lock_guard lock(mu_);
            return used_.insert(port).second;
        }

        void unregister(int port)
        {
            std::lock_guard lock(mu_);
            used_.erase(port);
        }

    private:
        std::mutex     mu_;
        std::set<int>  used_;
    };
#endif
}
