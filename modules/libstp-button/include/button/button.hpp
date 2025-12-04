//
// Created by eternalRose on 11/17/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif

#include "hal/Digital.hpp"

namespace libstp::button {

    class Button {
    private:
        static hal::digital::DigitalSensor* digital_sensor_;

        Button() = delete;

    public:
        Button(const Button&) = delete;
        Button& operator=(const Button&) = delete;
        Button(Button&&) = delete;
        Button& operator=(Button&&) = delete;

        static bool isPressed();

        static void setDigital(int port);

        static void waitForButtonPress();
    };

}