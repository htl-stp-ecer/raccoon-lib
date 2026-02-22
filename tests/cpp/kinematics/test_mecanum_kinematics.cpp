#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "kinematics/mecanum/mecanum.hpp"

using namespace libstp::test;

class MecanumKinematicsTest : public KinematicsTestFixture {};

TEST_F(MecanumKinematicsTest, GetWheelRadiusReturnsConstructorValue) {
    // Arrange
    constexpr double wheelbase = 0.2;    // 200mm front-to-back
    constexpr double trackWidth = 0.15;  // 150mm side-to-side
    constexpr double wheelRadius = 0.05; // 50mm

    libstp::kinematics::mecanum::MecanumKinematics kinematics(
        motors_[0].get(),  // front_left
        motors_[1].get(),  // front_right
        motors_[2].get(),  // back_left
        motors_[3].get(),  // back_right
        wheelbase,
        trackWidth,
        wheelRadius
    );

    // Act
    double result = kinematics.getWheelRadius();

    // Assert
    EXPECT_DOUBLE_EQ(result, wheelRadius);
}

TEST_F(MecanumKinematicsTest, GetWheelRadiusWithDifferentValue) {
    // Arrange
    constexpr double wheelbase = 0.25;
    constexpr double trackWidth = 0.20;
    constexpr double wheelRadius = 0.0375;  // 37.5mm - different wheel size

    libstp::kinematics::mecanum::MecanumKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        motors_[2].get(),
        motors_[3].get(),
        wheelbase,
        trackWidth,
        wheelRadius
    );

    // Act
    double result = kinematics.getWheelRadius();

    // Assert
    EXPECT_DOUBLE_EQ(result, wheelRadius);
}

TEST_F(MecanumKinematicsTest, WheelCountIsFour) {
    // Arrange
    libstp::kinematics::mecanum::MecanumKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        motors_[2].get(),
        motors_[3].get(),
        0.2,   // wheelbase
        0.15,  // trackWidth
        0.05   // wheelRadius
    );

    // Act & Assert
    EXPECT_EQ(kinematics.wheelCount(), 4);
}

TEST_F(MecanumKinematicsTest, SupportsLateralMotionIsTrue) {
    // Arrange
    libstp::kinematics::mecanum::MecanumKinematics kinematics(
        motors_[0].get(),
        motors_[1].get(),
        motors_[2].get(),
        motors_[3].get(),
        0.2,
        0.15,
        0.05
    );

    // Act & Assert
    EXPECT_TRUE(kinematics.supportsLateralMotion());
}
