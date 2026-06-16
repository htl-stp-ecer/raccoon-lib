#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drive/drive.hpp"
#include "motion/directional_line_follow_motion.hpp"
#include "test_support/mock_imu.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::test;
using namespace libstp::motion;
using namespace libstp::drive;
using ::testing::NiceMock;

class DirectionalLineFollowMotionTest : public MotionTestFixture
{
protected:
    void SetUp() override
    {
        MotionTestFixture::SetUp();
        mock_imu_ = std::make_shared<NiceMock<MockIMU>>();
        mock_imu_->setupDefaults();

        auto kinematics_owned = std::make_unique<NiceMock<MockKinematics>>();
        kinematics_owned->setupAsMecanum();
        drive_ = std::make_unique<Drive>(
            std::move(kinematics_owned),
            ChassisVelocityControlConfig{},
            *mock_imu_);

        pid_config_ = defaultUnifiedMotionPidConfig();
        pid_config_.linear = AxisConstraints{0.5, 1.0, 1.0};
        pid_config_.lateral = AxisConstraints{0.4, 1.0, 1.0};
    }

    std::shared_ptr<DirectionalLineFollowMotion> makeMotion(DirectionalLineFollowMotionConfig config)
    {
        MotionContext ctx{*drive_, *mock_odometry_, pid_config_};
        return std::make_shared<DirectionalLineFollowMotion>(ctx, config);
    }

    void setWorldPose(double x, double y, double heading_rad)
    {
        libstp::foundation::Pose pose;
        pose.position = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 0.0f);
        pose.heading = static_cast<float>(heading_rad);
        mock_odometry_->setPose(pose);
        mock_odometry_->setHeading(heading_rad);
    }

    static constexpr double kDt = 0.01;

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_{};
};

TEST_F(DirectionalLineFollowMotionTest, AngularCorrectionUsesOmegaChannel)
{
    DirectionalLineFollowMotionConfig cfg;
    cfg.heading_speed = 0.6;
    cfg.kp = 1.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.0);
    motion->start();
    motion->setSensorError(0.25);
    motion->update(kDt);

    const auto& telemetry = motion->getTelemetry();
    ASSERT_FALSE(telemetry.empty());
    EXPECT_NEAR(telemetry.back().cmd_vx_mps, 0.3, 1e-6);
    EXPECT_NEAR(telemetry.back().cmd_vy_mps, 0.0, 1e-6);
    EXPECT_GT(telemetry.back().cmd_wz_radps, 0.0);
}

TEST_F(DirectionalLineFollowMotionTest, LateralCorrectionUsesVyAndHoldsHeading)
{
    DirectionalLineFollowMotionConfig cfg;
    cfg.heading_speed = 0.5;
    cfg.correction_mode = LineFollowCorrectionMode::Lateral;
    cfg.heading_hold = true;
    cfg.kp = 1.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.1);
    motion->start();
    setWorldPose(0.0, 0.0, 0.05);
    motion->setSensorError(0.2);
    motion->update(kDt);

    const auto& telemetry = motion->getTelemetry();
    ASSERT_FALSE(telemetry.empty());
    EXPECT_NEAR(telemetry.back().cmd_vx_mps, 0.25, 1e-6);
    EXPECT_GT(telemetry.back().cmd_vy_mps, 0.0);
    EXPECT_GT(telemetry.back().cmd_wz_radps, 0.0);
}

TEST_F(DirectionalLineFollowMotionTest, ForwardCorrectionUsesVxDuringLateralTravel)
{
    DirectionalLineFollowMotionConfig cfg;
    cfg.strafe_speed = -0.5;
    cfg.correction_mode = LineFollowCorrectionMode::Forward;
    cfg.kp = 1.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.0);
    motion->start();
    motion->setSensorError(-0.3);
    motion->update(kDt);

    const auto& telemetry = motion->getTelemetry();
    ASSERT_FALSE(telemetry.empty());
    EXPECT_LT(telemetry.back().cmd_vx_mps, 0.0);
    EXPECT_NEAR(telemetry.back().cmd_vy_mps, -0.2, 1e-6);
}

TEST_F(DirectionalLineFollowMotionTest, DistanceTargetFinishesWithoutExternalCondition)
{
    DirectionalLineFollowMotionConfig cfg;
    cfg.heading_speed = 0.4;
    cfg.distance_m = 0.5;
    cfg.has_distance_target = true;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.0);
    motion->start();
    setWorldPose(0.6, 0.0, 0.0);
    motion->update(kDt);

    EXPECT_TRUE(motion->isFinished());
    const auto& telemetry = motion->getTelemetry();
    ASSERT_FALSE(telemetry.empty());
    EXPECT_TRUE(telemetry.back().finished);
}
