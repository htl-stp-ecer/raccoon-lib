//
// Created by tobias on 6/1/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif

namespace libstp::hal::analog
{
    class AnalogSensor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline std::set<int> used_analog_ports{};
        static void registerAnalogPort(int port);
        static void unregisterAnalogPort(int port);
#endif

    public:
        int port;

        explicit AnalogSensor(int port);

        virtual ~AnalogSensor();

        virtual int read() const;
    };
}
