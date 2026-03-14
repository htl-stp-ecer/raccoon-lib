//
// Created by tobias on 6/1/25.
//
#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif
#include "foundation/motor.hpp"
#include "hal/IMotor.hpp"

namespace libstp::hal::motor
{
    /**
     * Concrete motor wrapper whose method bodies are provided by the selected
     * platform bundle.
     *
     * The public contract here is what higher-level modules and Python bindings
     * program against. Shared process-wide safety behavior, such as duplicate
     * port tracking and fail-safe shutdown hooks, lives in `libstp-hal`.
     */
    class Motor : public IMotor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline std::set<int> used_motor_ports{};

        static void registerMotorPort(int port);

        static void unregisterMotorPort(int port);
#endif

    public:
        /**
         * Construct a motor on the given port.
         *
         * `inverted` flips command and feedback sign conventions so callers can
         * keep chassis-level logic independent of wiring direction.
         */
        explicit Motor(int port, bool inverted, const foundation::MotorCalibration& calibration = {});

        ~Motor() override;

        /// Open-loop percent output in the range expected by the platform driver.
        void setSpeed(int percent) override;
        /// Closed-loop velocity target in firmware-specific BEMF units.
        void setVelocity(int velocity) override;
        /// Closed-loop absolute move in firmware-specific position units.
        void moveToPosition(int velocity, int goalPosition) override;
        /// Closed-loop relative move in firmware-specific position units.
        void moveRelative(int velocity, int deltaPosition) override;
        /// Return the current motor position using the driver's sign convention.
        [[nodiscard]] int getPosition() const override;
        /// Return the driver's current BEMF or velocity-like feedback value.
        [[nodiscard]] int getBemf() const override;
        /// Return whether the last position command has completed.
        [[nodiscard]] bool isDone() const override;

        /// Actively stop this motor using the platform's safest available path.
        void brake() override;
        /// Disable this motor completely (no power, no brake — free-spinning).
        void off() override;
        /// Reset the position counter to zero.
        void resetPositionCounter() override;

        [[nodiscard]] const foundation::MotorCalibration& getCalibration() const override;
        void setCalibration(const foundation::MotorCalibration& calibration) override;
        [[nodiscard]] int getPort() const override { return port_; }
        [[nodiscard]] bool isInverted() const override { return inverted_; }

        /// Put every motor managed by the active platform into its disabled state.
        static void disableAll();
        /// Re-enable globally disabled motors when the platform supports it.
        static void enableAll();

    private:
        int port_;
        bool inverted_;
        foundation::MotorCalibration calibration_;
    };
}
