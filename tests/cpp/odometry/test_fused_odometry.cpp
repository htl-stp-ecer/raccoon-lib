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

// TODO: Add more comprehensive tests for position integration, heading tracking, etc.
