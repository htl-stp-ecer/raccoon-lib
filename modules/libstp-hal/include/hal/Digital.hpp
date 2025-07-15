//
// Created by tobias on 6/1/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif

namespace libstp::hal::digital
{
    class DigitalSensor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline std::set<int> used_digital_ports{};
        static void registerDigitalPort(int port);
        static void unregisterDigitalPort(int port);
#endif
    public:
        int port;

        explicit DigitalSensor(int port);

        ~DigitalSensor();

        bool read() const;
    };
}
