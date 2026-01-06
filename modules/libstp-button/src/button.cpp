//
// Created by eternalRose on 11/17/25.
//
#include "button/button.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include "foundation/logging.hpp"

namespace libstp::button
{
    Button& Button::instance()
    {
        static Button instance;
        return instance;
    }

    Button::~Button() = default;

    void Button::setDigital(int port)
    {
        digital_sensor_ = std::make_unique<hal::digital::DigitalSensor>(port);
    }

    void Button::setDigital(std::unique_ptr<hal::digital::DigitalSensor> sensor)
    {
        digital_sensor_ = std::move(sensor);
    }

    bool Button::isPressed() const
    {
        if (!digital_sensor_)
        {
            throw std::runtime_error("Button digital sensor not initialized.");
        }
        return digital_sensor_->read();
    }

    void Button::waitForButtonPress() const
    {
        if (!digital_sensor_)
        {
            throw std::runtime_error("Button digital sensor not initialized.");
        }

        using namespace std::chrono_literals;

        // Wait until button is released (if currently held)
        while (isPressed())
        {
            std::this_thread::sleep_for(10ms);
        }

        // Wait until button is pressed
        while (!isPressed())
        {
            std::this_thread::sleep_for(10ms);
        }
    }
} // namespace libstp::button
