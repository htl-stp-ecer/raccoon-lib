#pragma once

#include "hal/PortRegistry.hpp"

namespace libstp::hal::digital
{
    /**
     * Digital input wrapper implemented by the active platform bundle.
     *
     * The wrapper only owns the requested port number and shared safety-check
     * bookkeeping. Platform drivers provide the actual I/O behavior.
     */
    class DigitalSensor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline detail::PortRegistry registry_{};
        static void registerDigitalPort(int port);
        static void unregisterDigitalPort(int port);
#endif
    public:
        int port;

        /// Create a digital input bound to a platform-defined port index.
        explicit DigitalSensor(int port);

        ~DigitalSensor();

        /// Return the current boolean state for the configured input.
        bool read() const;
    };
}
