#pragma once

#include <gmock/gmock.h>
#include "kinematics/kinematics.hpp"

namespace libstp::test
{
    class MockKinematics : public kinematics::IKinematics
    {
    public:
        MOCK_METHOD(std::size_t, wheelCount, (), (const, override));
        MOCK_METHOD(kinematics::MotorCommands, applyCommand,
                    (const foundation::ChassisVelocity&, double), (override));
        MOCK_METHOD(foundation::ChassisVelocity, estimateState, (), (const, override));
        MOCK_METHOD(void, hardStop, (), (override));
        MOCK_METHOD(bool, supportsLateralMotion, (), (const, override));
        MOCK_METHOD(void, resetEncoders, (), (override));
        MOCK_METHOD(std::vector<calibration::CalibrationResult>, calibrateMotors,
                    (const calibration::CalibrationConfig&), (override));
        MOCK_METHOD(double, getWheelRadius, (), (const, override));
        MOCK_METHOD(std::vector<hal::motor::IMotor*>, getMotors, (), (const, override));
        MOCK_METHOD(void, applyPowerCommand,
                    (const foundation::ChassisVelocity&, int), (override));

        // Configurable behavior helpers
        void setSupportsLateral(bool supports) {
            ON_CALL(*this, supportsLateralMotion()).WillByDefault(testing::Return(supports));
        }

        void setEstimatedState(const foundation::ChassisVelocity& state) {
            ON_CALL(*this, estimateState()).WillByDefault(testing::Return(state));
        }

        void setWheelCount(std::size_t count) {
            ON_CALL(*this, wheelCount()).WillByDefault(testing::Return(count));
        }

        void setWheelRadius(double radius) {
            ON_CALL(*this, getWheelRadius()).WillByDefault(testing::Return(radius));
        }

        // Default setup for differential drive
        void setupAsDifferential() {
            setWheelCount(2);
            setSupportsLateral(false);
            setWheelRadius(0.05); // 50mm default wheel radius
            setEstimatedState(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            ON_CALL(*this, applyCommand(testing::_, testing::_))
                .WillByDefault(testing::Return(kinematics::MotorCommands{}));
        }

        // Default setup for mecanum drive
        void setupAsMecanum() {
            setWheelCount(4);
            setSupportsLateral(true);
            setWheelRadius(0.05); // 50mm default wheel radius
            setEstimatedState(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            ON_CALL(*this, applyCommand(testing::_, testing::_))
                .WillByDefault(testing::Return(kinematics::MotorCommands{}));
        }
    };
}
