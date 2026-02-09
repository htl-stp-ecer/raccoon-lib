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

    odom.update(0.01);

    EXPECT_NEAR(odom.getHeading(), 0.0, 1e-6);
}

TEST_F(FusedOdometryTest, UpdatesPositionBasedOnKinematics) {
    libstp::foundation::ChassisVelocity v{};
    v.vx = 1.0;
    v.vy = 0.0;
    v.wz = 0.0;
    mock_kinematics_->setEstimatedState(v);

    FusedOdometry odom(mock_imu_, mock_kinematics_);

    odom.update(1.0);

    auto pose = odom.getPose();
    EXPECT_NEAR(pose.position.x(), 1.0, 1e-6);
    EXPECT_NEAR(pose.position.y(), 0.0, 1e-6);
}

TEST_F(FusedOdometryTest, DistanceFromOriginTracksPosition) {
    libstp::foundation::ChassisVelocity v{};
    v.vx = 1.0;
    v.vy = 0.0;
    v.wz = 0.0;
    mock_kinematics_->setEstimatedState(v);

    FusedOdometry odom(mock_imu_, mock_kinematics_);

    odom.update(1.0);

    auto dist = odom.getDistanceFromOrigin();
    EXPECT_NEAR(dist.forward, 1.0, 1e-6);
    EXPECT_NEAR(dist.straight_line, 1.0, 1e-6);
}

TEST_F(FusedOdometryTest, TransformToAndFromBodyFrame) {
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    Eigen::Quaternionf q(Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ()));
    ON_CALL(*mock_imu_, getOrientation()).WillByDefault(testing::Return(q));
    odom.update(0.01);

    Eigen::Vector3f world_vec(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f body_vec = odom.transformToBodyFrame(world_vec);
    Eigen::Vector3f reconstructed = odom.transformToWorldFrame(body_vec);

    EXPECT_NEAR(world_vec.x(), reconstructed.x(), 1e-6);
    EXPECT_NEAR(world_vec.y(), reconstructed.y(), 1e-6);
    EXPECT_NEAR(world_vec.z(), reconstructed.z(), 1e-6);
}

TEST_F(FusedOdometryTest, ResetWorksToGivenPose) {
    FusedOdometry odom(mock_imu_, mock_kinematics_);

    libstp::foundation::Pose new_pose;
    new_pose.position = Eigen::Vector3f(5.0f, 3.0f, 0.0f);
    new_pose.orientation = Eigen::Quaternionf::Identity();

    odom.reset(new_pose);

    auto pose = odom.getPose();
    EXPECT_NEAR(pose.position.x(), 5.0f, 1e-6);
    EXPECT_NEAR(pose.position.y(), 3.0f, 1e-6);
    EXPECT_NEAR(pose.position.z(), 0.0f, 1e-6);
}
