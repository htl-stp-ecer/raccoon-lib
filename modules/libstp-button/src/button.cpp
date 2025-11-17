//
// Created by eternalRose on 11/17/25.
//
#include "../include/button.hpp"

#include <bits/this_thread_sleep.h>

namespace libstp::button {

    void Button::setDigital(const hal::digital::DigitalSensor &sensor) {
        digital_sensor_ = sensor;
    }

    bool Button::isPressed() {
        return digital_sensor_.read();
    }

    void Button::waitForButtonPress() {
        while (!isPressed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(10)));
        }
    }
}
