//
// Created by eternalRose on 11/17/25.
//
#include "button/button.hpp"

#include <chrono>
#include <thread>

namespace libstp::button {

    hal::digital::DigitalSensor* Button::digital_sensor_ = nullptr;

    void Button::setDigital(int port) {
        if (digital_sensor_ != nullptr) {
            delete digital_sensor_;
        }
        digital_sensor_ = new hal::digital::DigitalSensor(port);
    }

    bool Button::isPressed() {
        if (digital_sensor_ == nullptr) {
            throw std::runtime_error("Button digital sensor not initialized.");
        }
        return digital_sensor_->read();
    }

    void Button::waitForButtonPress() {
        if (digital_sensor_ == nullptr) {
            throw std::runtime_error("Button digital sensor not initialized.");
        }

        while (!isPressed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
