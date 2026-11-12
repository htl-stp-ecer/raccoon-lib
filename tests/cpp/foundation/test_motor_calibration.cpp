#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "foundation/motor.hpp"

using libstp::foundation::MotorCalibration;

namespace
{
    MotorCalibration withTicksToRad(const double ticksToRad)
    {
        MotorCalibration cal{};
        cal.ticks_to_rad = ticksToRad;
        return cal;
    }
} // namespace

TEST(MotorCalibrationValidateTest, DefaultIsValid)
{
    EXPECT_NO_THROW(MotorCalibration{}.validate());
}

TEST(MotorCalibrationValidateTest, RejectsZeroNegativeAndNonFiniteTicksToRad)
{
    EXPECT_THROW(withTicksToRad(0.0).validate(), std::invalid_argument);
    EXPECT_THROW(withTicksToRad(-0.01).validate(), std::invalid_argument);
    EXPECT_THROW(withTicksToRad(std::numeric_limits<double>::quiet_NaN()).validate(),
                 std::invalid_argument);
    // +inf is > 0 but would collapse every degree->tick conversion to 0.
    EXPECT_THROW(withTicksToRad(std::numeric_limits<double>::infinity()).validate(),
                 std::invalid_argument);
}

TEST(MotorCalibrationValidateTest, AcceptsSmallPositiveTicksToRad)
{
    EXPECT_NO_THROW(withTicksToRad(1e-9).validate());
}

TEST(MotorCalibrationValidateDegreesTest, AcceptsNormalAngles)
{
    const auto cal = withTicksToRad(0.01);
    EXPECT_NO_THROW(cal.validate_degrees(0.0));
    EXPECT_NO_THROW(cal.validate_degrees(180.0));
    EXPECT_NO_THROW(cal.validate_degrees(-720.0));
}

TEST(MotorCalibrationValidateDegreesTest, RejectsNonFiniteAngle)
{
    const auto cal = withTicksToRad(0.01);
    EXPECT_THROW(cal.validate_degrees(std::numeric_limits<double>::quiet_NaN()),
                 std::invalid_argument);
    EXPECT_THROW(cal.validate_degrees(std::numeric_limits<double>::infinity()),
                 std::invalid_argument);
    EXPECT_THROW(cal.validate_degrees(-std::numeric_limits<double>::infinity()),
                 std::invalid_argument);
}

TEST(MotorCalibrationValidateDegreesTest, RejectsAngleBeyondTickRange)
{
    // 1 deg ~ 1.7e10 ticks > INT_MAX
    const auto cal = withTicksToRad(1e-12);
    EXPECT_THROW(cal.validate_degrees(90.0), std::invalid_argument);
    EXPECT_THROW(cal.validate_degrees(-90.0), std::invalid_argument);
}
