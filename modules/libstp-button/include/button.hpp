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
        static hal::digital::DigitalSensor digital_sensor_;
        Button() = default;

    public:
        Button(const Button&) = delete;
        Button& operator=(const Button&) = delete;
        Button(Button&&) = delete;
        Button& operator=(Button&&) = delete;

        ~Button() = default;

        static Button& getInstance() {
            static Button instance;
            return instance;
        }

        void setDigital(const hal::digital::DigitalSensor &sensor);
        bool isPressed();
        void waitForButtonPress();
    };

}