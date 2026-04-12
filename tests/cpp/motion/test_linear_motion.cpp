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

    static constexpr double kDt = 0.01;  // 100 Hz
    static constexpr double kPi = std::numbers::pi;

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_{};
};

// ===================================================================
// Relative heading (default behavior)
// ===================================================================

TEST_F(LinearMotionTest, Relative_DefaultConfigHasNoTargetHeading) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.25;
    EXPECT_FALSE(cfg.target_heading_rad.has_value());
}

TEST_F(LinearMotionTest, Relative_ZeroErrorWhenOnHeading) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;

    auto motion = makeMotion(cfg);
    motion->start();

    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    mock_odometry_->simulateForwardProgress(0.10);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.0, 1e-9);
}

TEST_F(LinearMotionTest, Relative_PositiveErrorWhenDriftedCW) {
    // Robot drifted clockwise (heading decreased). getHeadingError returns
    // positive (= need to turn CCW/left to correct). The heading PID output
    // should be positive omega (CCW).
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;

    auto motion = makeMotion(cfg);
    motion->start();

    mock_odometry_->setHeading(-0.1);
    mock_odometry_->setHeadingError(0.1);  // angularError: target - current > 0
    mock_odometry_->simulateForwardProgress(0.10);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.1, 0.01);
    // PID with positive error → positive omega → corrects CW drift by turning CCW
    EXPECT_GT(t.back().pid_heading_raw, 0.0)
        << "Positive error must produce positive omega (turn left) to correct CW drift";
}

TEST_F(LinearMotionTest, Relative_NegativeErrorWhenDriftedCCW) {
    // Robot drifted counter-clockwise. getHeadingError returns negative
    // (= need to turn CW/right). PID output should be negative omega.
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;

    auto motion = makeMotion(cfg);
    motion->start();

    mock_odometry_->setHeading(0.1);
    mock_odometry_->setHeadingError(-0.1);
    mock_odometry_->simulateForwardProgress(0.10);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, -0.1, 0.01);
    EXPECT_LT(t.back().pid_heading_raw, 0.0)
        << "Negative error must produce negative omega (turn right) to correct CCW drift";
}

TEST_F(LinearMotionTest, Relative_CompletesAtDistance) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.10;

    auto motion = makeMotion(cfg);
    motion->start();

    mock_odometry_->simulateForwardProgress(0.10);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);

    for (int i = 0; i < 20; ++i)
        motion->update(kDt);

    EXPECT_TRUE(motion->isFinished());
}

TEST_F(LinearMotionTest, Relative_IgnoresAbsoluteHeading) {
    // In relative mode, even wild absolute heading changes must not affect
    // the yaw error — only getHeadingError() matters.
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;

    auto motion = makeMotion(cfg);
    motion->start();

    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(99.0));
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    mock_odometry_->simulateForwardProgress(0.10);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.0, 1e-9)
        << "Relative mode must ignore absolute heading";
}

// ===================================================================
// Absolute heading (target_heading_rad set)
// ===================================================================

TEST_F(LinearMotionTest, Absolute_ConfigStoresTarget) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.30;
    cfg.target_heading_rad = 1.0;
    EXPECT_TRUE(cfg.target_heading_rad.has_value());
    EXPECT_DOUBLE_EQ(*cfg.target_heading_rad, 1.0);
}

TEST_F(LinearMotionTest, Absolute_ZeroErrorWhenOnTarget) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 0.5;

    auto motion = makeMotion(cfg);
    motion->start();

    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(0.5));
    mock_odometry_->simulateForwardProgress(0.05);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.0, 1e-9);
}

TEST_F(LinearMotionTest, Absolute_PositiveErrorWhenDriftedCW) {
    // Target = 1.0 rad. Robot drifted clockwise to 0.9 rad.
    // Error should be positive (need to turn CCW to reach target).
    // angularError convention: target - current = 1.0 - 0.9 = +0.1
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 1.0;

    auto motion = makeMotion(cfg);
    motion->start();

    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(0.9));
    mock_odometry_->simulateForwardProgress(0.10);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, 0.1, 0.001)
        << "target(1.0) - current(0.9) = +0.1";
    EXPECT_GT(t.back().pid_heading_raw, 0.0)
        << "Positive error → positive omega → turn CCW to correct CW drift";
}

TEST_F(LinearMotionTest, Absolute_NegativeErrorWhenDriftedCCW) {
    // Target = 1.0 rad. Robot drifted counter-clockwise to 1.1 rad.
    // Error should be negative (need to turn CW to reach target).
    // angularError convention: target - current = 1.0 - 1.1 = -0.1
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 1.0;

    auto motion = makeMotion(cfg);
    motion->start();

    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(1.1));
    mock_odometry_->simulateForwardProgress(0.10);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, -0.1, 0.001)
        << "target(1.0) - current(1.1) = -0.1";
    EXPECT_LT(t.back().pid_heading_raw, 0.0)
        << "Negative error → negative omega → turn CW to correct CCW drift";
}

TEST_F(LinearMotionTest, Absolute_CorrectionDirectionMatchesRelative) {
    // When the robot drifts the same way, absolute and relative mode must
    // produce the same sign of omega. This catches sign-flip bugs.
    const double drift = 0.15;  // drifted CCW by 0.15 rad

    // Relative mode: target = 0 (captured at start), current = +0.15
    LinearMotionConfig rel_cfg;
    rel_cfg.distance_m = 0.50;
    auto rel_motion = makeMotion(rel_cfg);
    rel_motion->start();
    mock_odometry_->setHeading(drift);
    mock_odometry_->setHeadingError(-drift);  // angularError(0.15, 0) = 0 - 0.15 = -0.15
    mock_odometry_->simulateForwardProgress(0.10);
    rel_motion->update(kDt);

    // Absolute mode: target = 2.0, current = 2.0 + 0.15 = 2.15
    LinearMotionConfig abs_cfg;
    abs_cfg.distance_m = 0.50;
    abs_cfg.target_heading_rad = 2.0;
    auto abs_motion = makeMotion(abs_cfg);
    abs_motion->start();
    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(2.0 + drift));
    mock_odometry_->simulateForwardProgress(0.10);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    abs_motion->update(kDt);

    const auto& rel_t = rel_motion->getTelemetry();
    const auto& abs_t = abs_motion->getTelemetry();
    ASSERT_FALSE(rel_t.empty());
    ASSERT_FALSE(abs_t.empty());

    // Both should have negative error (CCW drift → need CW correction)
    EXPECT_LT(rel_t.back().yaw_error_rad, 0.0);
    EXPECT_LT(abs_t.back().yaw_error_rad, 0.0);

    // Both should produce negative omega (turn CW)
    EXPECT_LT(rel_t.back().pid_heading_raw, 0.0);
    EXPECT_LT(abs_t.back().pid_heading_raw, 0.0);

    // Error magnitudes should be similar
    EXPECT_NEAR(std::abs(rel_t.back().yaw_error_rad),
                std::abs(abs_t.back().yaw_error_rad), 0.01);
}

TEST_F(LinearMotionTest, Absolute_WrapsAroundPiBoundary) {
    // Target near +pi, current just past -pi. Should take the short path.
    LinearMotionConfig cfg;
    cfg.distance_m = 0.50;
    cfg.target_heading_rad = 3.0;  // near +pi

    auto motion = makeMotion(cfg);
    motion->start();

    // Current = -3.0 → raw gap = 3.0 - (-3.0) = 6.0 → wrapped ≈ 6.0 - 2pi ≈ -0.283
    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(-3.0));
    mock_odometry_->simulateForwardProgress(0.05);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());

    double expected = std::remainder(3.0 - (-3.0), 2.0 * kPi);
    EXPECT_NEAR(t.back().yaw_error_rad, expected, 0.01);
    EXPECT_LT(std::abs(t.back().yaw_error_rad), kPi)
        << "Normalized error must be within [-pi, pi]";
}

TEST_F(LinearMotionTest, Absolute_NegativeTarget) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.30;
    cfg.target_heading_rad = -1.5;

    auto motion = makeMotion(cfg);
    motion->start();

    // Current = -1.3, target = -1.5 → error = -1.5 - (-1.3) = -0.2 (need CW)
    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(-1.3));
    mock_odometry_->simulateForwardProgress(0.05);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, -0.2, 0.01);
    EXPECT_LT(t.back().pid_heading_raw, 0.0)
        << "Robot is CCW of target → must turn CW (negative omega)";
}

TEST_F(LinearMotionTest, Absolute_LateralAxis) {
    LinearMotionConfig cfg;
    cfg.axis = LinearAxis::Lateral;
    cfg.distance_m = 0.20;
    cfg.target_heading_rad = 0.0;

    auto motion = makeMotion(cfg);
    motion->start();

    // Drifted CCW to +0.05 during strafe → error = 0 - 0.05 = -0.05
    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(0.05));
    mock_odometry_->simulateForwardProgress(0.0, 0.05);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);
    motion->update(kDt);

    const auto& t = motion->getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().yaw_error_rad, -0.05, 0.01);
    EXPECT_LT(t.back().pid_heading_raw, 0.0)
        << "CCW drift during strafe → CW correction (negative omega)";
}

TEST_F(LinearMotionTest, Absolute_CompletesAtDistance) {
    LinearMotionConfig cfg;
    cfg.distance_m = 0.10;
    cfg.target_heading_rad = 0.5;

    auto motion = makeMotion(cfg);
    motion->start();

    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(0.5));
    mock_odometry_->simulateForwardProgress(0.10);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);

    for (int i = 0; i < 20; ++i)
        motion->update(kDt);

    EXPECT_TRUE(motion->isFinished());
}

// ===================================================================
// Sustained correction: verify the PID consistently corrects toward
// target over multiple cycles (catches oscillation / sign-flip bugs)
// ===================================================================

TEST_F(LinearMotionTest, Absolute_SustainedCorrectionDoesNotOscillate) {
    // Simulate a robot that is 0.2 rad CW of the target heading.
    // Over 10 cycles the omega should stay consistently positive (correcting
    // toward target), not flip-flop in sign (which would indicate oscillation
    // from a sign bug).
    LinearMotionConfig cfg;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 1.0;

    auto motion = makeMotion(cfg);
    motion->start();

    // Constant heading error: robot stuck at 0.8 rad, target 1.0 rad
    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(0.8));
    mock_odometry_->simulateForwardProgress(0.05);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);

    for (int i = 0; i < 10; ++i)
        motion->update(kDt);

    const auto& telem = motion->getTelemetry();
    int positive_count = 0;
    int negative_count = 0;
    for (const auto& t : telem) {
        if (t.pid_heading_raw > 0.001) ++positive_count;
        if (t.pid_heading_raw < -0.001) ++negative_count;
    }

    EXPECT_GT(positive_count, 0)
        << "Should produce positive omega to correct CW drift";
    EXPECT_EQ(negative_count, 0)
        << "Should never produce negative omega — that would be oscillation "
           "from a sign bug (driving away from target then snapping back)";
}

TEST_F(LinearMotionTest, Absolute_LargeDriftDoesNotReverse) {
    // Even with a large heading error (~45 degrees), omega should stay
    // consistently positive, not oscillate.
    LinearMotionConfig cfg;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;

    auto motion = makeMotion(cfg);
    motion->start();

    // Robot is at -0.78 rad (~45 degrees CW of target)
    ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(-0.78));
    mock_odometry_->simulateForwardProgress(0.05);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);

    for (int i = 0; i < 10; ++i)
        motion->update(kDt);

    const auto& telem = motion->getTelemetry();
    for (const auto& t : telem) {
        EXPECT_GT(t.yaw_error_rad, 0.0)
            << "Error must stay positive (need CCW correction)";
        EXPECT_GE(t.pid_heading_raw, 0.0)
            << "Omega must not go negative (would drive away from target)";
    }
}
