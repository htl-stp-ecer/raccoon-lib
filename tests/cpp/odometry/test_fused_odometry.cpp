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

    odom.update(0.01); // Capture initial IMU orientation as origin

    // Simulate a 90° CCW yaw rotation around Z axis.
    // DMP uses ENU convention: positive rotation around Z = CCW from above.
    mock_imu_->setOrientation(Eigen::Quaternionf(
        Eigen::AngleAxisf(static_cast<float>(kHalfPi), Eigen::Vector3f::UnitZ())));
    odom.update(0.01);

    EXPECT_EQ(mock_imu_->getYawRateAxisMode(), libstp::hal::imu::TurnAxisMode::WorldZ);
    EXPECT_NEAR(odom.getHeading(), kHalfPi, 1e-3);
}

TEST_F(FusedOdometryTest, HeadingTracksTiltedBodyRotation) {
    // This test verifies the critical fix: heading extraction works correctly
    // when the robot body is tilted (e.g. body Y pointing up instead of Z).
    //
    // Bug: extractYaw(Q_initial⁻¹ * Q_current) returns ~0 for tilted bodies
    // because the relative quaternion encodes heading change as pitch (R_y),
    // not yaw (R_z).
    //
    // Fix: extract yaw from absolute quaternions independently and subtract.
    constexpr double kHalfPi = 1.5707963267948966;
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    // Body Y points up: tilt by -90° around X axis
    // Q_initial = R_x(-π/2): body Y → earth Z
    const Eigen::Quaternionf q_tilt(
        Eigen::AngleAxisf(static_cast<float>(-kHalfPi), Eigen::Vector3f::UnitX()));
    mock_imu_->setOrientation(q_tilt);
    odom.update(0.01); // Capture tilted initial orientation

    // Simulate 90° CCW turn around world Z while body is tilted.
    // Q_current = R_z(+π/2) * R_x(-π/2) (ENU: +π/2 for CCW)
    const Eigen::Quaternionf q_turn(
        Eigen::AngleAxisf(static_cast<float>(kHalfPi), Eigen::Vector3f::UnitZ()));
    mock_imu_->setOrientation(q_turn * q_tilt);
    odom.update(0.01);

    EXPECT_NEAR(odom.getHeading(), kHalfPi, 1e-3);
}

TEST_F(FusedOdometryTest, TurnAxisConfigApplied) {
    FusedOdometryConfig config;
    config.turn_axis = "body_y";
    FusedOdometry odom(mock_imu_, mock_kinematics_, config);

    EXPECT_EQ(mock_imu_->getYawRateAxisMode(), libstp::hal::imu::TurnAxisMode::BodyY);
}
