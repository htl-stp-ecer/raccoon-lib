//
// Created by tobias on 6/1/25.
//
#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif

namespace libstp::hal::motor
{
    class Motor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static std::set<int> used_motor_ports;

        static void registerMotorPort(int port);

        static void unregisterMotorPort(int port);
#endif

    public:
        int port;
        bool inverted;

        explicit Motor(int port, bool inverted);

        ~Motor();

        void setSpeed(int percent) const;
        int getSpeed() const;

        void brake() const;

        static void disableAll();
    };

    class EncoderMotor : public Motor
    {
        int getPosition() const;

        void moveToPosition(int position);

        void resetPosition();
    };
}
