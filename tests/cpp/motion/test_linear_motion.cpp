#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <numbers>

#include "test_support/test_fixtures.hpp"
#include "test_support/mock_imu.hpp"
#include "motion/linear_motion.hpp"
#include "drive/drive.hpp"

using namespace libstp::test;
using namespace libstp::motion;
using namespace libstp::drive;
using ::testing::NiceMock;
using ::testing::Return;

// ---------------------------------------------------------------------------
// Fixture: builds a real Drive with mock kinematics/IMU + mock odometry,
// then constructs a LinearMotion from them.
//
// LinearMotion is now ABSOLUTE-ONLY: target_heading_rad is required (defaults
// to 0.0), the controller no longer calls odometry().reset() in start(), and
// it projects the current pose into the body frame captured at start.
// ---------------------------------------------------------------------------

class LinearMotionTest : public MotionTestFixture
{
protected:
    void SetUp() override {
        MotionTestFixture::SetUp();
        mock_imu_ = std::make_shared<NiceMock<MockIMU>>();
        mock_imu_->setupDefaults();

        auto kinematics_owned = std::make_unique<NiceMock<MockKinematics>>();
        kinematics_owned->setupAsDifferential();
        drive_ = std::make_unique<Drive>(
            std::move(kinematics_owned),
            ChassisVelocityControlConfig{},
            *mock_imu_);
    }

    std::shared_ptr<LinearMotion> makeMotion(LinearMotionConfig config) {
        pid_config_ = defaultUnifiedMotionPidConfig();
        pid_config_.linear = AxisConstraints{0.5, 1.0, 1.0};
        pid_config_.lateral = AxisConstraints{0.3, 0.8, 0.8};

        MotionContext ctx{*drive_, *mock_odometry_, pid_config_};
        return std::make_shared<LinearMotion>(ctx, config);
    }

    /// Set the mock odometry pose using world-frame x/y/heading. The motion
    /// projects subsequent reads into the body frame captured at start().
    void setWorldPose(double x, double y, double heading_rad) {
        libstp::foundation::Pose pose;
        pose.position = Eigen::Vector3f(static_cast<float>(x),
                                         static_cast<float>(y),
                                         0.0f);
        pose.heading = static_cast<float>(heading_rad);
        mock_odometry_->setPose(pose);
        mock_odometry_->setHeading(heading_rad);
        ON_CALL(*mock_odometry_, getAbsoluteHeading())
            .WillByDefault(Return(heading_rad));
    }

    static constexpr double kDt = 0.01;  // 100 Hz
    static constexpr double kPi = std::numbers::pi;

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_{};
};

// ===================================================================
// Config defaults
// ===================================================================

TEST_F(LinearMotionTest, ConfigDefaultTargetHeadingIsZero) {
    LinearMotionConfig cfg;
    EXPECT_DOUBLE_EQ(cfg.target_heading_rad, 0.0);
}

// ===================================================================
// start() must NOT reset odometry (Phase 4 absolute-only contract).
// ===================================================================

TEST_F(LinearMotionTest, StartDoesNotResetOdometry) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 0.0;

    auto motion = makeMotion(cfg);

    EXPECT_CALL(*mock_odometry_, reset()).Times(0);
    setWorldPose(1.5, 0.7, 0.3);
    motion->start();
}

// ===================================================================
// Body-frame projection: forward axis
// ===================================================================

TEST_F(LinearMotionTest, ForwardAxis_TraveledFromInitialPoseIsZeroAtStart) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 0.0;

    auto motion = makeMotion(cfg);
    setWorldPose(2.0, 1.0, 0.0);
    motion->start();
    // Pose unchanged after start: traveled = 0 ⇒ distance_error = full target
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().position_m, 0.0, 1e-6);
    EXPECT_NEAR(t.back().distance_error_m, 0.50, 1e-6);
}

TEST_F(LinearMotionTest, ForwardAxis_HeadingZero_PoseMovedAlongX) {
    // Robot starts at (1,1, 0 rad). Pose advances to (1.10, 1.0, 0 rad).
    // body_forward = +0.10, body_lateral = 0.
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 0.0;

    auto motion = makeMotion(cfg);
    setWorldPose(1.0, 1.0, 0.0);
    motion->start();

    setWorldPose(1.10, 1.0, 0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().position_m, 0.10, 1e-5);
    EXPECT_NEAR(t.back().cross_track_m, 0.0, 1e-5);
}

TEST_F(LinearMotionTest, ForwardAxis_HeadingNonZero_ProjectsIntoBodyFrame) {
    // Robot starts at (0,0, π/2 rad). World-x maps to body-right (lateral).
    // Pose advances to (0.0, 0.20, π/2). World-Δ = (0, +0.20).
    // body_forward = 0*cos(π/2) + 0.20*sin(π/2) = 0.20
    // body_lateral = -0*sin(π/2) + 0.20*cos(π/2) = 0
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = kPi / 2.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, kPi / 2.0);
    motion->start();

    setWorldPose(0.0, 0.20, kPi / 2.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().position_m, 0.20, 1e-5);
    EXPECT_NEAR(t.back().cross_track_m, 0.0, 1e-5);
}

// ===================================================================
// Heading error: yaw_error_rad uses the absolute target.
// ===================================================================

TEST_F(LinearMotionTest, HeadingError_ZeroWhenOnTarget) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 0.5;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.5);
    motion->start();
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.0, 1e-9);
}

TEST_F(LinearMotionTest, HeadingError_PositiveWhenDriftedCW) {
    // target=1.0, current=0.9 → error = 1.0 - 0.9 = +0.1 (turn CCW)
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 1.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.9);
    motion->start();
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.1, 1e-6);
    EXPECT_GT(t.back().pid_heading_raw, 0.0)
        << "Positive error → positive omega → CCW correction";
}

TEST_F(LinearMotionTest, HeadingError_NegativeWhenDriftedCCW) {
    // target=1.0, current=1.1 → error = -0.1 (turn CW)
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 1.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 1.1);
    motion->start();
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, -0.1, 1e-6);
    EXPECT_LT(t.back().pid_heading_raw, 0.0);
}

TEST_F(LinearMotionTest, HeadingError_WrapsAroundPiBoundary) {
    // Target near +π, current just past −π. Should take the short path.
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 3.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, -3.0);
    motion->start();
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    const double expected = std::remainder(3.0 - (-3.0), 2.0 * kPi);
    EXPECT_NEAR(t.back().yaw_error_rad, expected, 1e-6);
    EXPECT_LT(std::abs(t.back().yaw_error_rad), kPi);
}

TEST_F(LinearMotionTest, HeadingError_NegativeTarget) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.30;
    cfg.target_heading_rad = -1.5;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, -1.3);
    motion->start();
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, -0.2, 1e-6);
    EXPECT_LT(t.back().pid_heading_raw, 0.0);
}

// ===================================================================
// Lateral axis
// ===================================================================

TEST_F(LinearMotionTest, LateralAxis_ProjectsLateralIntoPrimary) {
    // Robot at (0,0, 0 rad). After motion: (0, -0.10, 0 rad) in world → body
    // body_forward = 0, body_lateral = -0*sin(0) + (-0.10)*cos(0) = -0.10
    // ...wait, lateral = -dx*sinθ + dy*cosθ. With dy = -0.10 in world,
    // body_lateral = -0 + (-0.10)*1 = -0.10.
    // Recall odometry "lateral" is right-positive but the +y axis math gives
    // (-sinθ, cosθ) which at θ=0 is (0, +1) so a +y world delta → +lateral.
    // Therefore robot strafing right should produce body_lateral > 0; that
    // means world-y must be +0.10.
    LinearMotionConfig cfg;
    cfg.axis = LinearAxis::Lateral;
    cfg.distance_m = 0.20;
    cfg.target_heading_rad = 0.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.0);
    motion->start();

    setWorldPose(0.0, 0.10, 0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().position_m, 0.10, 1e-5);
}

// ===================================================================
// Completion criterion uses body-frame traveled distance, not raw odometry.
// ===================================================================

TEST_F(LinearMotionTest, CompletesAtDistance) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.10;
    cfg.target_heading_rad = 0.5;

    auto motion = makeMotion(cfg);
    setWorldPose(2.0, 3.0, 0.5);
    motion->start();

    // Pose advances along body-forward by exactly the target.
    setWorldPose(2.0 + 0.10 * std::cos(0.5),
                 3.0 + 0.10 * std::sin(0.5),
                 0.5);

    for (int i = 0; i < 20; ++i)
        motion->update(kDt);

    EXPECT_TRUE(motion->isFinished());
}

// ===================================================================
// Sustained correction: verify no oscillation under constant drift.
// ===================================================================

TEST_F(LinearMotionTest, SustainedCorrectionDoesNotOscillate) {
    LinearMotionConfig cfg;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 1.0;

    auto motion = makeMotion(cfg);
    setWorldPose(0.0, 0.0, 0.8);
    motion->start();

    // Hold the robot at 0.8 rad with non-trivial forward progress.
    setWorldPose(0.05 * std::cos(0.8), 0.05 * std::sin(0.8), 0.8);

    for (int i = 0; i < 10; ++i)
        motion->update(kDt);

    const auto& telem = motion->getTelemetry();
    int positive_count = 0;
    int negative_count = 0;
    for (const auto& t : telem) {
        if (t.pid_heading_raw > 0.001) ++positive_count;
        if (t.pid_heading_raw < -0.001) ++negative_count;
    }

    EXPECT_GT(positive_count, 0);
    EXPECT_EQ(negative_count, 0)
        << "Constant CW drift must produce monotonically positive omega";
}
