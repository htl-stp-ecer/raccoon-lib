#pragma once

#include "button.hpp"

namespace libstp::button {

    class ButtonWrapper {
    public:
        ButtonWrapper() = default;
        ~ButtonWrapper() = default;

        bool isPressed() {
            return Button::getInstance().isPressed();
        }

        void setDigital(hal::digital::DigitalSensor sensor) {
            Button::getInstance().setDigital(sensor);
        }

        void waitForButtonPress() {
            Button::getInstance().waitForButtonPress();
        }
    };

}