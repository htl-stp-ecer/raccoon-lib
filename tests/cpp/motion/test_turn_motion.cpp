#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <numbers>

#include "drive/drive.hpp"
#include "motion/motion_config.hpp"
#include "motion/turn_motion.hpp"
#include "test_support/mock_imu.hpp"
#include "test_support/mock_kinematics.hpp"
#include "test_support/mock_odometry.hpp"

using namespace libstp::motion;
using namespace libstp::drive;
using namespace libstp::foundation;
using namespace libstp::kinematics;
using namespace libstp::test;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::AtLeast;

static constexpr double kPi = std::numbers::pi;
static constexpr double kDt = 0.01;

class TurnMotionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mock_imu_ = std::make_shared<NiceMock<MockIMU>>();
        mock_imu_->setupDefaults();
        mock_odometry_ = std::make_shared<NiceMock<MockOdometry>>();
        mock_odometry_->setupDefaults();

        auto kinematics = std::make_unique<NiceMock<MockKinematics>>();
        kinematics->setupAsDifferential();
        drive_ = std::make_unique<Drive>(std::move(kinematics), ChassisVelocityControlConfig{}, *mock_imu_);

        pid_config_ = defaultUnifiedMotionPidConfig();
        pid_config_.angular = AxisConstraints{2.0, 4.0, 4.0};
    }

    TurnMotion makeTurn(double target_rad, bool has_target = true)
    {
        MotionContext ctx{*drive_, *mock_odometry_, pid_config_};
        TurnConfig cfg;
        cfg.target_angle_rad = target_rad;
        cfg.has_angle_target = has_target;
        return TurnMotion(ctx, cfg);
    }

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::shared_ptr<NiceMock<MockOdometry>> mock_odometry_;
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_;
};

// ---------------------------------------------------------------------------
// Lifecycle basics
// ---------------------------------------------------------------------------

TEST_F(TurnMotionTest, NotFinishedBeforeFirstUpdate)
{
    auto turn = makeTurn(kPi / 2.0);
    EXPECT_FALSE(turn.isFinished());
}

TEST_F(TurnMotionTest, TargetZero_FinishesAfterOneUpdate)
{
    // heading never moves → accumulated delta = 0 = target → error 0 AND velocity 0
    auto turn = makeTurn(0.0);
    ON_CALL(*mock_odometry_, getHeading()).WillByDefault(Return(0.0));

    turn.update(kDt);

    EXPECT_TRUE(turn.isFinished());
}

TEST_F(TurnMotionTest, UpdateCallsOdometryUpdateEachTick)
{
    auto turn = makeTurn(0.0);
    EXPECT_CALL(*mock_odometry_, update(kDt)).Times(AtLeast(1));
    turn.update(kDt);
}

// ---------------------------------------------------------------------------
// Convergence
// ---------------------------------------------------------------------------

TEST_F(TurnMotionTest, ConvergesWhenAtTarget)
{
    // start() captures prev_heading = 0.0; tick 1 returns 0.5 → accumulated = 0.5 = target.
    // Heading stays at 0.5 → velocity decays to zero over ~20 ticks.
    auto turn = makeTurn(0.5);
    EXPECT_CALL(*mock_odometry_, getHeading())
        .WillOnce(Return(0.0))
        .WillRepeatedly(Return(0.5));

    for (int i = 0; i < 60 && !turn.isFinished(); ++i)
        turn.update(kDt);

    EXPECT_TRUE(turn.isFinished());
}

TEST_F(TurnMotionTest, DoesNotFinishWhenFarFromTarget)
{
    auto turn = makeTurn(kPi / 2.0);
    ON_CALL(*mock_odometry_, getHeading()).WillByDefault(Return(0.0));

    for (int i = 0; i < 5; ++i)
        turn.update(kDt);

    EXPECT_FALSE(turn.isFinished());
}

// ---------------------------------------------------------------------------
// Heading accumulation across ±π wraparound
// ---------------------------------------------------------------------------

TEST_F(TurnMotionTest, AccumulatesHeadingAcrossWrapBoundary)
{
    // Robot at 3.0 rad turns +0.5 rad CCW. New heading = 3.5 rad wraps to 3.5 - 2π ≈ -2.78 rad.
    // Naive delta = -2.78 - 3.0 = -5.78 (completely wrong).
    // angularError(3.0, -2.78) correctly yields +0.5 rad.
    const double start_heading = 3.0;
    const double end_heading   = 3.0 + 0.5 - 2.0 * kPi;

    auto turn = makeTurn(0.5);

    EXPECT_CALL(*mock_odometry_, getHeading())
        .WillOnce(Return(start_heading))
        .WillRepeatedly(Return(end_heading));

    for (int i = 0; i < 60 && !turn.isFinished(); ++i)
        turn.update(kDt);

    EXPECT_TRUE(turn.isFinished());
}

// ---------------------------------------------------------------------------
// hasReachedAngle
// ---------------------------------------------------------------------------

TEST_F(TurnMotionTest, HasReachedAngle_FalseBeforeStart)
{
    auto turn = makeTurn(kPi / 4.0);
    EXPECT_FALSE(turn.hasReachedAngle());
}

TEST_F(TurnMotionTest, HasReachedAngle_TrueWhenWithinTolerance)
{
    const double target = pid_config_.angle_tolerance_rad / 2.0;
    auto turn = makeTurn(target);
    ON_CALL(*mock_odometry_, getHeading()).WillByDefault(Return(target));

    for (int i = 0; i < 60 && !turn.isFinished(); ++i)
        turn.update(kDt);

    EXPECT_TRUE(turn.hasReachedAngle());
}

TEST_F(TurnMotionTest, HasReachedAngle_FalseWhenFarFromTarget)
{
    auto turn = makeTurn(kPi / 2.0);
    ON_CALL(*mock_odometry_, getHeading()).WillByDefault(Return(0.0));

    turn.update(kDt);

    EXPECT_FALSE(turn.hasReachedAngle());
}

// ---------------------------------------------------------------------------
// hardStop delegation
// ---------------------------------------------------------------------------

TEST_F(TurnMotionTest, SuppressHardStop_KinematicsHardStopNotCalled)
{
    auto owned = std::make_unique<testing::StrictMock<MockKinematics>>();
    auto* km = owned.get();
    owned->setupAsDifferential();
    EXPECT_CALL(*km, applyCommand(testing::_, testing::_)).WillRepeatedly(Return(MotorCommands{}));
    EXPECT_CALL(*km, estimateState()).WillRepeatedly(Return(ChassisVelocity{}));
    EXPECT_CALL(*km, hardStop()).Times(0);

    auto strict_drive = std::make_unique<Drive>(std::move(owned), ChassisVelocityControlConfig{}, *mock_imu_);
    MotionContext ctx{*strict_drive, *mock_odometry_, pid_config_};
    TurnConfig cfg;
    cfg.target_angle_rad = 0.0;
    TurnMotion turn(ctx, cfg);
    turn.setSuppressHardStopOnComplete(true);

    ON_CALL(*mock_odometry_, getHeading()).WillByDefault(Return(0.0));
    turn.update(kDt);

    EXPECT_TRUE(turn.isFinished());
}

TEST_F(TurnMotionTest, HardStop_CalledOnCompletion)
{
    auto owned = std::make_unique<testing::StrictMock<MockKinematics>>();
    auto* km = owned.get();
    owned->setupAsDifferential();
    EXPECT_CALL(*km, applyCommand(testing::_, testing::_)).WillRepeatedly(Return(MotorCommands{}));
    EXPECT_CALL(*km, estimateState()).WillRepeatedly(Return(ChassisVelocity{}));
    EXPECT_CALL(*km, hardStop()).Times(1);

    auto strict_drive = std::make_unique<Drive>(std::move(owned), ChassisVelocityControlConfig{}, *mock_imu_);
    MotionContext ctx{*strict_drive, *mock_odometry_, pid_config_};
    TurnConfig cfg;
    cfg.target_angle_rad = 0.0;
    TurnMotion turn(ctx, cfg);

    ON_CALL(*mock_odometry_, getHeading()).WillByDefault(Return(0.0));
    turn.update(kDt);
}
