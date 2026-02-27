#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "kinematics/differential/differential.hpp"

using namespace libstp::test;

class DifferentialKinematicsTest : public KinematicsTestFixture {};

TEST_F(DifferentialKinematicsTest, GetWheelRadiusReturnsConstructorValue) {
    constexpr double wheelbase = 0.2;
    constexpr double wheelRadius = 0.05;
    // Arrange

    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        wheelbase,
        wheelRadius
    );

    double result = kinematics.getWheelRadius();

    EXPECT_DOUBLE_EQ(result, wheelRadius);
}

TEST_F(DifferentialKinematicsTest, GetWheelRadiusWithDifferentValue) {
    // Arrange
    constexpr double wheelbase = 0.15;
    constexpr double wheelRadius = 0.0325;  // 32.5mm - typical small wheel

    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        wheelbase,
        wheelRadius
    );

    // Act
    double result = kinematics.getWheelRadius();

    // Assert
    EXPECT_DOUBLE_EQ(result, wheelRadius);
}

TEST_F(DifferentialKinematicsTest, WheelCountIsTwo) {
    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        0.2,  // wheelbase
        0.05  // wheelRadius
    );

    EXPECT_EQ(kinematics.wheelCount(), 2);
}

TEST_F(DifferentialKinematicsTest, SupportsLateralMotionIsFalse) {
    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        0.2,
        0.05
    );

    EXPECT_FALSE(kinematics.supportsLateralMotion());
}