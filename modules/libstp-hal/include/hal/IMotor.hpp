//
// IMotor interface for dependency injection and testing
//
#pragma once

#include "foundation/motor.hpp"

namespace libstp::hal::motor
{
    /**
     * @brief Interface for motor abstraction
     *
     * This interface allows for mocking motors in unit tests
     * and enables dependency injection in kinematics classes.
     */
    struct IMotor
    {
        virtual ~IMotor() = default;

        /// Open-loop percent output.
        virtual void setSpeed(int percent) = 0;
        /// Set a velocity target in firmware-specific BEMF units.
        virtual void setVelocity(int velocity) = 0;
        /// Move to an absolute position using firmware-side control.
        virtual void moveToPosition(int velocity, int goalPosition) = 0;
        /// Move by a relative position delta using firmware-side control.
        virtual void moveRelative(int velocity, int deltaPosition) = 0;
        [[nodiscard]] virtual int getPosition() const = 0;
        [[nodiscard]] virtual int getBemf() const = 0;
        [[nodiscard]] virtual bool isDone() const = 0;
        /// Stop this motor using the implementation's preferred strategy.
        virtual void brake() = 0;
        /// Disable this motor completely (no power, no brake — free-spinning).
        virtual void off() = 0;
        /// Reset the position counter to zero.
        virtual void resetPositionCounter() = 0;

        [[nodiscard]] virtual const foundation::MotorCalibration& getCalibration() const = 0;
        virtual void setCalibration(const foundation::MotorCalibration& calibration) = 0;
        [[nodiscard]] virtual int getPort() const = 0;
        [[nodiscard]] virtual bool isInverted() const = 0;

        /**
         * @brief Push per-motor firmware-side velocity PID gains to the driver.
         *
         * Default no-op so platforms that don't expose a per-motor firmware PID
         * channel keep compiling unchanged. The wombat platform forwards to
         * LcmDataWriter::setMotorPid(); the mock platform stores the gains for
         * test introspection.
         */
        virtual void setFirmwarePidGains(float kp, float ki, float kd)
        {
            (void)kp;
            (void)ki;
            (void)kd;
        }

        /**
         * @brief Read back the gains last passed to setFirmwarePidGains().
         *
         * Default no-op (returns zeros) so platforms that don't store the
         * gains keep compiling unchanged. Override in concrete platforms
         * (wombat, mock) to expose the cached value — primarily useful in
         * tuner/test code that needs to revert to a previous gain set.
         */
        virtual void getLastFirmwarePidGains(float& kp, float& ki, float& kd) const
        {
            kp = 0.0f;
            ki = 0.0f;
            kd = 0.0f;
        }
    };
}
