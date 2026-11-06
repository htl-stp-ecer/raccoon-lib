#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "test_support/mock_motor.hpp"

using libstp::foundation::MotorCalibration;
using libstp::test::MockMotor;
using testing::StrictMock;

namespace
{
    constexpr double kPi = std::numbers::pi;

    MotorCalibration calibrationWithTicksToRad(const double ticksToRad)
    {
        MotorCalibration cal{};
        cal.ticks_to_rad = ticksToRad;
        return cal;
    }
} // namespace

TEST(MotorDegreesTest, MoveToDegreesConvertsToAbsoluteTicks)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(0.01));

    // 180 deg = pi rad -> 314.159 ticks -> 314
    EXPECT_CALL(motor, getCalibration()).Times(1);
    EXPECT_CALL(motor, moveToPosition(800, std::lround(kPi / 0.01)));
    motor.moveToDegrees(800, 180.0);
}

TEST(MotorDegreesTest, MoveToDegreesNegativeAngle)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(kPi / 180.0)); // 1 tick == 1 deg

    EXPECT_CALL(motor, getCalibration()).Times(1);
    EXPECT_CALL(motor, moveToPosition(500, -45));
    motor.moveToDegrees(500, -45.0);
}

TEST(MotorDegreesTest, MoveRelativeDegreesConvertsDelta)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(kPi / 180.0));

    EXPECT_CALL(motor, getCalibration()).Times(1);
    EXPECT_CALL(motor, moveRelative(600, 90));
    motor.moveRelativeDegrees(600, 90.0);
}

TEST(MotorDegreesTest, ZeroCalibrationThrowsAndSendsNothing)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(0.0));

    EXPECT_CALL(motor, getCalibration()).Times(2);
    EXPECT_THROW(motor.moveToDegrees(500, 90.0), std::invalid_argument);
    EXPECT_THROW(motor.moveRelativeDegrees(500, 90.0), std::invalid_argument);
}
