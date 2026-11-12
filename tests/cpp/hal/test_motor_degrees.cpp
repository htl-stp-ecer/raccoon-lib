#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

TEST(MotorDegreesTest, NonFiniteAngleThrowsAndSendsNothing)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(0.01));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_CALL(motor, getCalibration()).Times(4);
    EXPECT_THROW(motor.moveToDegrees(500, nan), std::invalid_argument);
    EXPECT_THROW(motor.moveToDegrees(500, -inf), std::invalid_argument);
    EXPECT_THROW(motor.moveRelativeDegrees(500, nan), std::invalid_argument);
    EXPECT_THROW(motor.moveRelativeDegrees(500, inf), std::invalid_argument);
}

TEST(MotorDegreesTest, NonFiniteCalibrationThrowsAndSendsNothing)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(std::numeric_limits<double>::quiet_NaN()));

    EXPECT_CALL(motor, getCalibration()).Times(1);
    EXPECT_THROW(motor.moveToDegrees(500, 90.0), std::invalid_argument);

    motor.setCalibration(calibrationWithTicksToRad(std::numeric_limits<double>::infinity()));
    EXPECT_CALL(motor, getCalibration()).Times(1);
    EXPECT_THROW(motor.moveRelativeDegrees(500, 90.0), std::invalid_argument);
}

TEST(MotorDegreesTest, AngleBeyondTickRangeThrows)
{
    StrictMock<MockMotor> motor;
    motor.setCalibration(calibrationWithTicksToRad(1e-12)); // 1 deg ~ 1.7e10 ticks > INT_MAX

    EXPECT_CALL(motor, getCalibration()).Times(1);
    EXPECT_THROW(motor.moveToDegrees(500, 90.0), std::invalid_argument);
}
