//
// Created by tobias on 6/1/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif

namespace libstp::hal::servo
{
    /**
     * Servo output wrapper implemented by the active platform bundle.
     *
     * The wrapper caches the last commanded position locally so callers can
     * query `getPosition()` even when the underlying driver is write-only.
     */
    class Servo
    {
        float storedPosition = 0.0f;

#ifdef SAFETY_CHECKS_ENABLED
        static inline std::set<int> used_servo_ports{};

        static void registerServoPort(int port);

        static void unregisterServoPort(const int port);
#endif

    public:
        int port;

        /// Create a servo bound to a platform-defined port index.
        explicit Servo(int port);

        ~Servo();

        /// Command a new servo position in degrees (0-180).
        void setPosition(float position);
        /// Return the last commanded position in degrees.
        float getPosition() const;
        /// Command a smooth move to targetDeg at the given speed; interpolation runs in the C++ reader.
        void setSmoothPosition(float targetDeg, float speedDegPerSec, int easing);

        /// Enable output while preserving the last commanded position.
        void enable() const;
        /// Disable output while preserving the last commanded position.
        void disable() const;

        /// Force every servo into the platform's fully disabled state.
        static void fullyDisableAll();
    };
}
