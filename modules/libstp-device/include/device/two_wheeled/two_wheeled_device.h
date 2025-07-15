//
// Created by tobias on 12/26/24.
//

#pragma once
#include <tuple>

#include "datatype/axis.hpp"
#include "device/device.hpp"
#include "hal/Motor.hpp"

namespace libstp::device::two_wheeled
{
    class TwoWheeledDevice final : public Device
    {
    public:
        hal::motor::Motor leftMotor, rightMotor;

        mutable float ticksPerRevolution = 1582; // Measure this
        mutable float wheelBase = 0.1796; // Measure this
        mutable float wheelRadius = 0.035; // Measure this

        int lastLeftTicks = 0;
        int initialLeftTicks = 0;

        int lastRightTicks = 0;
        int initialRightTicks = 0;

        TwoWheeledDevice(const datatype::Axis orientation,
                         const datatype::Direction direction,
                         const hal::motor::Motor& left_motor,
                         const hal::motor::Motor& right_motor)
            : Device(orientation, direction),
              leftMotor(left_motor),
              rightMotor(right_motor)
        {
        }

    protected:
        void initializeKinematicDriveController() override;
        std::tuple<float, float, float> computeMaxSpeeds() override;
        void applyKinematicsModel(const datatype::AbsoluteSpeed& speed) override;
        std::tuple<float, float, float> getWheelVelocities(float dtSeconds) override;
        void stopDevice() override;
        [[nodiscard]] std::pair<float, float> computeDrivenDistance() const override;
    };
}
