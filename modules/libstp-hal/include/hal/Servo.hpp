//
// Created by tobias on 6/1/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif

namespace libstp::hal::servo
{
    class Servo
    {
        int storedPosition = 0;

#ifdef SAFETY_CHECKS_ENABLED
        static std::set<int> used_servo_ports;

        static void registerServoPort(int port);

        static void unregisterServoPort(const int port);
#endif

    public:
        int port;

        explicit Servo(int port);

        ~Servo();

        void setPosition(int position);
        int getPosition() const;

        void enable() const;
        void disable() const;

        static void fullyDisableAll();
    };
}
