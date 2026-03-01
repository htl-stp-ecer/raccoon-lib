#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "odometry/fused/fused_odometry.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::test;
using namespace libstp::odometry::fused;

class FusedOdometryTest : public OdometryTestFixture {};

TEST_F(FusedOdometryTest, ConstructsSuccessfully) {
    EXPECT_NO_THROW({
        FusedOdometry odom(mock_imu_, mock_kinematics_);
    });
}

TEST_F(FusedOdometryTest, InitialPoseAtOrigin) {
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    auto pose = odom.getPose();
    EXPECT_NEAR(pose.position.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.position.z(), 0.0f, 1e-6f);
}

TEST_F(FusedOdometryTest, InitialHeadingZero) {
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    // First update to initialize IMU
    odom.update(0.01);

    EXPECT_NEAR(odom.getHeading(), 0.0, 1e-6);
}

TEST_F(FusedOdometryTest, HeadingTracksYawRotation) {
    constexpr double kHalfPi = 1.5707963267948966;
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    odom.update(0.01); // Capture initial IMU heading as origin

    // Simulate a 90 deg CCW rotation via firmware heading
    mock_imu_->setHeading(static_cast<float>(kHalfPi));
    odom.update(0.01);

    EXPECT_EQ(mock_imu_->getYawRateAxisMode(), libstp::hal::imu::TurnAxisMode::WorldZ);
    EXPECT_NEAR(odom.getHeading(), kHalfPi, 1e-3);
}

TEST_F(FusedOdometryTest, HeadingTracksTiltedBodyRotation) {
    // Heading comes directly from firmware, so tilt is irrelevant.
    constexpr double kHalfPi = 1.5707963267948966;
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    mock_imu_->setHeading(0.0f);
    odom.update(0.01); // Capture initial heading

    // Simulate 90 deg CCW turn — firmware reports heading directly
    mock_imu_->setHeading(static_cast<float>(kHalfPi));
    odom.update(0.01);

    EXPECT_NEAR(odom.getHeading(), kHalfPi, 1e-3);
}

TEST_F(FusedOdometryTest, TurnAxisConfigApplied) {
    FusedOdometryConfig config;
    config.turn_axis = "body_y";
    FusedOdometry odom(mock_imu_, mock_kinematics_, config);

    EXPECT_EQ(mock_imu_->getYawRateAxisMode(), libstp::hal::imu::TurnAxisMode::BodyY);
}
