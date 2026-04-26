//
// Created by tobias on 6/1/25.
//

#pragma once

#include "hal/PortRegistry.hpp"

namespace libstp::hal::analog
{
    /**
     * Thin analog input wrapper implemented by the selected platform bundle.
     *
     * The constructor stores the requested port number and, when safety checks
     * are enabled, participates in duplicate-port detection shared across
     * platforms.
     */
    class AnalogSensor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline detail::PortRegistry registry_{};
        static void registerAnalogPort(int port);
        static void unregisterAnalogPort(int port);
#endif

    public:
        int port;

        /// Create an analog sensor bound to a platform-defined port index.
        explicit AnalogSensor(int port);

        virtual ~AnalogSensor();

        /// Read the current raw analog value from the selected platform.
        virtual int read() const;
    };
}
