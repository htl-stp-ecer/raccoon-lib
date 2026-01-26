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

        virtual void setSpeed(int percent) = 0;
        [[nodiscard]] virtual int getPosition() const = 0;
        virtual void brake() = 0;

        [[nodiscard]] virtual const foundation::MotorCalibration& getCalibration() const = 0;
        virtual void setCalibration(const foundation::MotorCalibration& calibration) = 0;
        [[nodiscard]] virtual int getPort() const = 0;
        [[nodiscard]] virtual bool isInverted() const = 0;
    };
}
