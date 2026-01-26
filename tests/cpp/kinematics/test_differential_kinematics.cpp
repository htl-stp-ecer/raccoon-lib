#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "kinematics/differential/differential.hpp"

using namespace libstp::test;

class DifferentialKinematicsTest : public KinematicsTestFixture {};

TEST_F(DifferentialKinematicsTest, GetWheelRadiusReturnsConstructorValue) {
    // Arrange
    constexpr double wheelbase = 0.2;  // 200mm
    constexpr double wheelRadius = 0.05;  // 50mm
    constexpr double maxVel = 1.0;
    constexpr double maxAccel = 2.0;

    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        wheelbase,
        wheelRadius,
        maxVel,
        maxAccel
    );

    // Act
    double result = kinematics.getWheelRadius();

    // Assert
    EXPECT_DOUBLE_EQ(result, wheelRadius);
}

TEST_F(DifferentialKinematicsTest, GetWheelRadiusWithDifferentValue) {
    // Arrange
    constexpr double wheelbase = 0.15;
    constexpr double wheelRadius = 0.0325;  // 32.5mm - typical small wheel
    constexpr double maxVel = 0.5;
    constexpr double maxAccel = 1.0;

    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        wheelbase,
        wheelRadius,
        maxVel,
        maxAccel
    );

    // Act
    double result = kinematics.getWheelRadius();

    // Assert
    EXPECT_DOUBLE_EQ(result, wheelRadius);
}

TEST_F(DifferentialKinematicsTest, WheelCountIsTwo) {
    // Arrange
    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        0.2,  // wheelbase
        0.05, // wheelRadius
        1.0,  // maxVel
        2.0   // maxAccel
    );

    // Act & Assert
    EXPECT_EQ(kinematics.wheelCount(), 2);
}

TEST_F(DifferentialKinematicsTest, SupportsLateralMotionIsFalse) {
    // Arrange
    libstp::kinematics::differential::DifferentialKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        0.2,
        0.05,
        1.0,
        2.0
    );

    // Act & Assert
    EXPECT_FALSE(kinematics.supportsLateralMotion());
}
