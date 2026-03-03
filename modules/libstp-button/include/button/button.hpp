//
// Created by eternalRose on 11/17/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif
#include <memory>
#include "hal/Digital.hpp"

namespace libstp::button {

    /** Singleton wrapper around the robot's shared physical button input. */
    class Button final {
    public:
        /** Access the process-wide button instance. */
        static Button& instance();

        // Set up the digital sensor using a port number (creates the sensor internally).
        void setDigital(int port);
        // Alternatively inject an already-created sensor (helps in tests).
        void setDigital(std::unique_ptr<hal::digital::DigitalSensor> sensor);

        /** Return the current raw pressed state. */
        bool isPressed() const;

        /** Block until the button is pressed according to the active sensor implementation. */
        void waitForButtonPress() const;

        Button(const Button&) = delete;
        Button& operator=(const Button&) = delete;
        Button(Button&&) = delete;
        Button& operator=(Button&&) = delete;

    private:
        Button() = default;
        ~Button();

        std::unique_ptr<hal::digital::DigitalSensor> digital_sensor_;
    };
} // namespace libstp::button
