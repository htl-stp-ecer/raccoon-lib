#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cmath>
#include <numbers>

#include "drive/drive.hpp"
#include "kinematics/kinematics.hpp"
#include "motion/diagonal_motion.hpp"
#include "motion/motion_config.hpp"
#include "foundation/speed_mode_context.hpp"
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

namespace
{
constexpr double kPi = std::numbers::pi;
constexpr double kDt = 0.01;
}

// Distinctive suite prefix: DiagonalMotionCov
class DiagonalMotionCovTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Ensure SpeedMode is off (process-wide singleton).
        SpeedModeContext::instance().setSpeedModeEnabled(false);

        mock_imu_ = std::make_shared<NiceMock<MockIMU>>();
        mock_imu_->setupDefaults();
        mock_odometry_ = std::make_shared<NiceMock<MockOdometry>>();
        mock_odometry_->setupDefaults();

        auto kinematics = std::make_unique<NiceMock<MockKinematics>>();
        kinematics->setupAsDifferential();
        mock_kin_ = kinematics.get();  // retained for saturation injection
        drive_ = std::make_unique<Drive>(std::move(kinematics), ChassisVelocityControlConfig{}, *mock_imu_);

        pid_config_ = defaultUnifiedMotionPidConfig();
        pid_config_.linear = AxisConstraints{1.0, 2.0, 2.0};
        pid_config_.angular = AxisConstraints{2.0, 4.0, 4.0};
    }

    void TearDown() override
    {
        SpeedModeContext::instance().setSpeedModeEnabled(false);
    }

    DiagonalMotion makeDiagonal(DiagonalMotionConfig cfg)
    {
        MotionContext ctx{*drive_, *mock_odometry_, pid_config_};
        return DiagonalMotion(ctx, cfg);
    }

    void setWorldPose(double x, double y, double heading_rad)
    {
        Pose pose;
        pose.position = Eigen::Vector3f(static_cast<float>(x),
                                        static_cast<float>(y), 0.0f);
        pose.heading = static_cast<float>(heading_rad);
        mock_odometry_->setPose(pose);
        ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(heading_rad));
    }

    // Force the drivetrain to report (or stop reporting) actuator saturation,
    // which drives the speed/heading-scale derating + recovery logic.
    void setSaturated(bool saturated)
    {
        MotorCommands mc;
        mc.saturated_any = saturated;
        mc.saturation_mask = saturated ? 0x3u : 0u;
        ON_CALL(*mock_kin_, applyCommand(::testing::_, ::testing::_))
            .WillByDefault(Return(mc));
    }

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::shared_ptr<NiceMock<MockOdometry>> mock_odometry_;
    NiceMock<MockKinematics>* mock_kin_{nullptr};
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_;
};

// ===================================================================
// Lifecycle
// ===================================================================

TEST_F(DiagonalMotionCovTest, NotFinishedBeforeUpdate)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.5;
    auto m = makeDiagonal(cfg);
    EXPECT_FALSE(m.isFinished());
}

TEST_F(DiagonalMotionCovTest, StartDoesNotResetOdometry)
{
    DiagonalMotionConfig cfg;
    cfg.distance_m = 0.5;
    auto m = makeDiagonal(cfg);
    EXPECT_CALL(*mock_odometry_, reset()).Times(0);
    setWorldPose(1.0, 2.0, 0.3);
    m.start();
}

TEST_F(DiagonalMotionCovTest, UpdateCallsOdometryUpdate)
{
    DiagonalMotionConfig cfg;
    cfg.distance_m = 0.5;
    auto m = makeDiagonal(cfg);
    setWorldPose(0.0, 0.0, 0.0);
    EXPECT_CALL(*mock_odometry_, update(kDt)).Times(AtLeast(1));
    m.update(kDt);
}

TEST_F(DiagonalMotionCovTest, NonPositiveDtIsNoOp)
{
    DiagonalMotionConfig cfg;
    cfg.distance_m = 0.5;
    auto m = makeDiagonal(cfg);
    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    m.update(0.0);
    EXPECT_TRUE(m.getTelemetry().empty());
}

// ===================================================================
// SpeedMode gate: distance-target motion must reject SpeedMode.
// ===================================================================

TEST_F(DiagonalMotionCovTest, StartThrowsWhenSpeedModeActiveWithDistanceTarget)
{
    DiagonalMotionConfig cfg;
    cfg.distance_m = 0.5;
    cfg.has_distance_target = true;
    auto m = makeDiagonal(cfg);

    SpeedModeContext::instance().setSpeedModeEnabled(true);
    setWorldPose(0.0, 0.0, 0.0);
    EXPECT_THROW(m.start(), std::logic_error);
}

TEST_F(DiagonalMotionCovTest, StartOkWhenSpeedModeActiveWithoutDistanceTarget)
{
    DiagonalMotionConfig cfg;
    cfg.distance_m = 0.0;
    cfg.has_distance_target = false;  // until-only mode
    auto m = makeDiagonal(cfg);

    SpeedModeContext::instance().setSpeedModeEnabled(true);
    setWorldPose(0.0, 0.0, 0.0);
    EXPECT_NO_THROW(m.start());
}

// ===================================================================
// Body-frame projection onto rotated travel frame
// ===================================================================

// angle=0: primary axis = body forward, cross = body lateral.
TEST_F(DiagonalMotionCovTest, AngleZeroForwardMapsToPrimary)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.5;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setWorldPose(0.20, 0.0, 0.0);  // +0.20 along world x = body forward
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    EXPECT_NEAR(t.position_m, 0.20, 1e-5);
    EXPECT_NEAR(t.cross_track_m, 0.0, 1e-5);
    EXPECT_NEAR(t.distance_error_m, 0.30, 1e-5);
}

// angle=45deg: a +world-x move of 0.2 m projects onto travel as 0.2*cos45,
// cross-track = -0.2*sin45 (perpendicular component).
TEST_F(DiagonalMotionCovTest, FortyFiveDegreeProjection)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = kPi / 4.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setWorldPose(0.20, 0.0, 0.0);
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    // body_forward=0.20, body_lateral=0
    // primary = 0.20*cos45 + 0 = 0.1414 ; cross = -0.20*sin45 = -0.1414
    EXPECT_NEAR(t.position_m, 0.20 * std::cos(kPi / 4.0), 1e-5);
    EXPECT_NEAR(t.cross_track_m, -0.20 * std::sin(kPi / 4.0), 1e-5);
}

// A pure-lateral travel direction (angle=pi/2): forward world move maps to
// cross-track, while body-lateral maps to primary.
TEST_F(DiagonalMotionCovTest, NinetyDegreeLateralProjection)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = kPi / 2.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    // Move +0.2 in world y => body_lateral = +0.2, body_forward = 0
    setWorldPose(0.0, 0.20, 0.0);
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    // primary = 0*cos90 + 0.2*sin90 = 0.2 ; cross = -0*sin90 + 0.2*cos90 = 0
    EXPECT_NEAR(t.position_m, 0.20, 1e-5);
    EXPECT_NEAR(t.cross_track_m, 0.0, 1e-5);
}

// Projection respects the initial (start) heading frame, not world axes.
TEST_F(DiagonalMotionCovTest, ProjectsIntoInitialHeadingFrame)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = kPi / 2.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, kPi / 2.0);  // facing +y world
    m.start();
    // Robot moves forward (its +x body == +y world) by 0.15.
    setWorldPose(0.0, 0.15, kPi / 2.0);
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    EXPECT_NEAR(t.position_m, 0.15, 1e-5);
    EXPECT_NEAR(t.cross_track_m, 0.0, 1e-5);
}

// Nonzero START position AND nonzero start heading: exercises the dx/dy
// subtraction against initial_position_ and the cos_h/sin_h rotation exactly.
// Start at world (1.0, 2.0, heading=pi/3). Robot drives body-forward by 0.30.
// World delta = (0.30*cos60, 0.30*sin60); body_forward must read 0.30 and
// body_lateral 0 (so primary=0.30 at travel angle 0).
TEST_F(DiagonalMotionCovTest, NonzeroStartPoseAndHeadingProjectsExactly)
{
    const double h = kPi / 3.0;  // 60 deg
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = h;
    auto m = makeDiagonal(cfg);

    setWorldPose(1.0, 2.0, h);
    m.start();
    setWorldPose(1.0 + 0.30 * std::cos(h), 2.0 + 0.30 * std::sin(h), h);
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    EXPECT_NEAR(t.position_m, 0.30, 1e-5);      // forward progress
    EXPECT_NEAR(t.cross_track_m, 0.0, 1e-5);    // no perpendicular drift
}

// A pure cross-track displacement (perpendicular to travel) with a nonzero
// travel angle: pins the cross_track sign/terms at line 129.
TEST_F(DiagonalMotionCovTest, CrossTrackSignForPerpendicularDrift)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = kPi / 2.0;   // travel = body-right
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    // Move +0.10 along body-forward (world +x). For travel angle pi/2, forward
    // is perpendicular to travel => cross_track = -body_forward*sin(90) = -0.10.
    setWorldPose(0.10, 0.0, 0.0);
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    EXPECT_NEAR(t.position_m, 0.0, 1e-5);          // no progress along travel
    EXPECT_NEAR(t.cross_track_m, -0.10, 1e-5);     // negative perpendicular
}

// ===================================================================
// Velocity command rotation back into robot frame
// ===================================================================

// angle=0 forward command => vx>0, vy~0.
TEST_F(DiagonalMotionCovTest, ForwardCommandIsPureVx)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    m.update(kDt);  // far from goal => positive primary cmd

    const auto& t = m.getTelemetry().back();
    EXPECT_GT(t.cmd_vx_mps, 0.0);
    EXPECT_NEAR(t.cmd_vy_mps, 0.0, 1e-9);
}

// angle=45deg: vx and vy components equal (cmd*cos45 == cmd*sin45).
TEST_F(DiagonalMotionCovTest, DiagonalCommandSplitsEqually)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = kPi / 4.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    EXPECT_GT(t.cmd_vx_mps, 0.0);
    EXPECT_GT(t.cmd_vy_mps, 0.0);
    EXPECT_NEAR(t.cmd_vx_mps, t.cmd_vy_mps, 1e-9);  // 45 deg => equal split
}

// ===================================================================
// Heading PID: yaw error from absolute target
// ===================================================================

TEST_F(DiagonalMotionCovTest, YawErrorPositiveDriftCW)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 1.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.9);  // 0.1 below target
    m.start();
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    EXPECT_NEAR(t.yaw_error_rad, 0.1, 1e-6);
    EXPECT_GT(t.cmd_wz_radps, 0.0);  // CCW correction
}

TEST_F(DiagonalMotionCovTest, YawErrorWrapsAroundPi)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 3.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, -3.0);
    m.start();
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    const double expected = std::remainder(3.0 - (-3.0), 2.0 * kPi);
    EXPECT_NEAR(t.yaw_error_rad, expected, 1e-6);
    EXPECT_LT(std::abs(t.yaw_error_rad), kPi);
}

// ===================================================================
// Max velocity scaling
// ===================================================================

TEST_F(DiagonalMotionCovTest, PrimaryCommandClampedToScaledMaxVelocity)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 10.0;     // far => max primary command
    cfg.speed_scale = 0.5;     // max_velocity = 0.5 * 1.0 = 0.5
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    for (int i = 0; i < 40; ++i)
        m.update(kDt);

    double peak = 0.0;
    for (const auto& t : m.getTelemetry())
    {
        EXPECT_LE(t.cmd_vx_mps, 0.5 + 1e-9);  // angle 0 => vx == primary
        peak = std::max(peak, t.cmd_vx_mps);
    }
    // Held at start far from goal, the profile cruises at the scaled cap
    // (0.5 * linear.max_velocity = 0.5 * 1.0). Pin the exact value so a wrong
    // scale arithmetic is caught.
    EXPECT_NEAR(peak, 0.5, 1e-6);
}

// ===================================================================
// Completion semantics
// ===================================================================

TEST_F(DiagonalMotionCovTest, CompletesAtTargetWhenSettled)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.10;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    // Hold at exactly target => error 0, velocity settles to ~0.
    setWorldPose(0.10, 0.0, 0.0);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);

    EXPECT_TRUE(m.isFinished());
}

// until-only mode never self-completes on distance even when "at" the target.
TEST_F(DiagonalMotionCovTest, UntilOnlyModeDoesNotCompleteOnDistance)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.0;
    cfg.has_distance_target = false;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    for (int i = 0; i < 20; ++i)
        m.update(kDt);

    EXPECT_FALSE(m.isFinished());
}

// Finished branch: zero command, no telemetry growth.
TEST_F(DiagonalMotionCovTest, FinishedUpdateIsNoGrowth)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.10;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setWorldPose(0.10, 0.0, 0.0);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);
    ASSERT_TRUE(m.isFinished());

    const size_t n = m.getTelemetry().size();
    m.update(kDt);
    EXPECT_EQ(m.getTelemetry().size(), n);
}

// ===================================================================
// Saturation derating: speed_scale ramps down under sustained saturation.
// ===================================================================

// Sustained saturation with a large yaw error derates speed_scale first
// (geometric 0.9 decay), then clamps at saturation_min_scale.
TEST_F(DiagonalMotionCovTest, SaturationDeratesSpeedScaleThenHeadingScale)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;  // sustained large yaw error
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setSaturated(true);

    for (int i = 0; i < 60; ++i)
        m.update(kDt);

    const auto& telem = m.getTelemetry();
    ASSERT_FALSE(telem.empty());
    const auto& last = telem.back();

    // speed_scale should have collapsed to its floor.
    EXPECT_NEAR(last.speed_scale, pid_config_.saturation_min_scale, 1e-6);
    // After speed_scale bottoms out, heading_scale begins derating below 1.0
    // toward its own floor.
    EXPECT_LT(last.heading_scale, 1.0);
    EXPECT_GE(last.heading_scale, pid_config_.heading_min_scale - 1e-6);

    // The derate is monotone non-increasing while saturated.
    for (size_t i = 1; i < telem.size(); ++i)
        EXPECT_LE(telem[i].speed_scale, telem[i - 1].speed_scale + 1e-9);
}

// First-step derate is exactly one geometric factor (1.0 -> 0.9).
TEST_F(DiagonalMotionCovTest, FirstSaturationStepDeratesByFactor)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setSaturated(true);
    m.update(kDt);

    const auto& t = m.getTelemetry();
    ASSERT_FALSE(t.empty());
    // Telemetry records the scale BEFORE this cycle's adjustment, so it reads
    // 1.0 on cycle 1. Run one more cycle to observe the applied derate.
    m.update(kDt);
    EXPECT_NEAR(m.getTelemetry().back().speed_scale,
                pid_config_.saturation_derating_factor, 1e-9);
}

// Recovery: once saturation clears and heading is on-target for the hold
// window, scales ramp back up toward 1.0.
TEST_F(DiagonalMotionCovTest, RecoveryRampsScalesBackUp)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    // Phase 1: saturate to drive speed_scale down.
    setSaturated(true);
    for (int i = 0; i < 30; ++i)
        m.update(kDt);
    const double derated = m.getTelemetry().back().speed_scale;
    ASSERT_LT(derated, 1.0);

    // Phase 2: clear saturation AND put heading on target (yaw_error ~ 0 <
    // heading_recovery_error_rad) so the recovery branch fires after the hold.
    setSaturated(false);
    setWorldPose(0.0, 0.0, 1.5);  // heading == target => yaw_error 0
    for (int i = 0; i < 30; ++i)
        m.update(kDt);

    const double recovered = m.getTelemetry().back().speed_scale;
    EXPECT_GT(recovered, derated);          // ramped up
    EXPECT_LE(recovered, 1.0 + 1e-9);       // never above 1
}

// ===================================================================
// Hardened: max velocity = speed_scale * linear.max_velocity (L19).
// Use a NON-UNIT linear.max_velocity so '*'->'/' is distinguishable
// (0.5*1.6=0.8 vs 0.5/1.6=0.3125). The primary command cruises at the
// scaled cap when held far from goal.
// ===================================================================
TEST_F(DiagonalMotionCovTest, MaxVelocityIsScaleTimesLinearMaxVelocity)
{
    pid_config_.linear = AxisConstraints{1.6, 2.0, 2.0};  // max_v = 1.6
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;     // far => command saturates at the cap
    cfg.speed_scale = 0.5;      // cap = 0.5 * 1.6 = 0.8
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    for (int i = 0; i < 60; ++i)
        m.update(kDt);

    double peak = 0.0;
    for (const auto& t : m.getTelemetry())
    {
        EXPECT_LE(t.cmd_vx_mps, 0.8 + 1e-9);
        peak = std::max(peak, t.cmd_vx_mps);
    }
    EXPECT_NEAR(peak, 0.8, 1e-6);
}

// ===================================================================
// Hardened cross-track projection (L129): cross = -bf*sin + bl*cos.
// Use angle=pi/4 AND a displacement with BOTH body_forward and body_lateral
// nonzero so the '+'->'-' mutant changes the result.
//   bf=0.10, bl=0.20, angle=45deg:
//   cross = -0.10*sin45 + 0.20*cos45 = (-0.10+0.20)*0.7071 = 0.07071
//   mutant '-': -0.10*sin45 - 0.20*cos45 = -0.2121
// ===================================================================
TEST_F(DiagonalMotionCovTest, CrossTrackSumBothComponents)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = kPi / 4.0;
    cfg.distance_m = 1.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    // heading 0 => body_forward = world dx, body_lateral = world dy.
    setWorldPose(0.10, 0.20, 0.0);
    m.update(kDt);

    const auto& t = m.getTelemetry().back();
    const double s = std::sin(kPi / 4.0), c = std::cos(kPi / 4.0);
    EXPECT_NEAR(t.cross_track_m, -0.10 * s + 0.20 * c, 1e-6);
    EXPECT_NEAR(t.position_m, 0.10 * c + 0.20 * s, 1e-6);  // primary unchanged
}

// ===================================================================
// Hardened filtered velocity (L132 /dt, L133 blend). angle=0, heading 0:
// primary = world dx. Move +0.20 over dt=0.01 from rest of the filter:
//   raw = 0.20/0.01 = 20 ; filtered = 0.3*20 = 6.0.
// Kills L132 ' / '->' * ' (raw -> 0.20*0.01) and L133 ' * '->' / '.
// ===================================================================
TEST_F(DiagonalMotionCovTest, FilteredVelocityFirstCycleExact)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;       // far => never completes here
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setWorldPose(0.20, 0.0, 0.0);
    m.update(kDt);

    EXPECT_NEAR(m.getTelemetry().back().filtered_velocity_mps, 6.0, 1e-6);
}

// Cycle 2 holds position: filtered = 0.3*0 + 0.7*6.0 = 4.2.
// Kills L133 ' + '->' - ' (would give -4.2).
TEST_F(DiagonalMotionCovTest, FilteredVelocitySecondCycleBlend)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setWorldPose(0.20, 0.0, 0.0);
    m.update(kDt);               // filtered -> 6.0
    m.update(kDt);               // hold => raw 0 ; filtered -> 0.7*6.0 = 4.2

    EXPECT_NEAR(m.getTelemetry().back().filtered_velocity_mps, 4.2, 1e-6);
}

// ===================================================================
// Hardened: commanded omega = pid_heading_raw * heading_scale_ (L166).
// Under sustained saturation heading_scale_ derates below 1.0, so the
// '*'->'/' mutant becomes observable: cmd_wz must equal raw * scale, not
// raw / scale. Verified directly from telemetry on every cycle.
// ===================================================================
TEST_F(DiagonalMotionCovTest, OmegaCommandIsRawTimesHeadingScale)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;   // sustained large yaw error
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setSaturated(true);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);

    bool saw_derated = false;
    for (const auto& t : m.getTelemetry())
    {
        EXPECT_NEAR(t.cmd_wz_radps, t.pid_heading_raw * t.heading_scale, 1e-9);
        if (t.heading_scale < 1.0) saw_derated = true;
    }
    EXPECT_TRUE(saw_derated);  // ensure heading_scale != 1 was exercised
}

// ===================================================================
// Hardened recovery logic: BOTH guard '&&' operators (L235, L243) and the
// heading_scale recovery '+' (L249). Recovery only fires when saturation is
// CLEARED *and* heading is on target *and* the hold window has elapsed *and*
// a scale is below the recovery threshold. We first derate, then clear and
// hold on-target and confirm heading_scale ramps UP by heading_recovery_rate.
// ===================================================================
TEST_F(DiagonalMotionCovTest, RecoveryRampsHeadingScaleByRecoveryRate)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    // Derate both scales hard.
    setSaturated(true);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);
    const double hs_derated = m.getTelemetry().back().heading_scale;
    ASSERT_LT(hs_derated, 1.0);

    // Clear saturation + put heading exactly on target so the recovery branch
    // fires after saturation_hold_cycles. Capture the heading_scale trajectory.
    setSaturated(false);
    setWorldPose(0.0, 0.0, 1.5);
    double prev = hs_derated;
    bool saw_increase_by_rate = false;
    for (int i = 0; i < 30; ++i)
    {
        m.update(kDt);
        const double hs = m.getTelemetry().back().heading_scale;
        if (hs > prev + 1e-9)
        {
            // Each recovery step adds exactly heading_recovery_rate (until clamp).
            EXPECT_NEAR(hs - prev, pid_config_.heading_recovery_rate, 1e-6);
            saw_increase_by_rate = true;
        }
        prev = hs;
    }
    EXPECT_TRUE(saw_increase_by_rate);
}

// L235 ' && '->' || ' guard: when saturation is CLEARED but heading error is
// LARGE (above recovery error), recovery must NOT fire. Under '&&' the branch
// is gated by both conditions; under '||' a large-error-but-unsaturated cycle
// would wrongly trigger recovery. After derating, clear saturation but keep a
// large yaw error => scales must NOT ramp back up.
TEST_F(DiagonalMotionCovTest, NoRecoveryWhenUnsaturatedButHeadingErrorLarge)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    setSaturated(true);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);
    const double sp_derated = m.getTelemetry().back().speed_scale;
    const double hs_derated = m.getTelemetry().back().heading_scale;

    // Unsaturated, but keep heading FAR from target (yaw_error 1.5 >> recovery err).
    setSaturated(false);
    setWorldPose(0.0, 0.0, 0.0);  // heading 0, target 1.5 => yaw_error 1.5
    for (int i = 0; i < 30; ++i)
        m.update(kDt);

    // No recovery: scales stay at their derated floor (the '&&' gate blocks it).
    EXPECT_NEAR(m.getTelemetry().back().speed_scale, sp_derated, 1e-9);
    EXPECT_NEAR(m.getTelemetry().back().heading_scale, hs_derated, 1e-9);
}

// L243 ' && '->' || ': recovery requires can_recover AND needs_recovery.
// During the hold window (unsaturated_cycles < saturation_hold_cycles) we have
// can_recover=false but needs_recovery=true (scales were derated). Under '&&'
// NO recovery happens until the hold elapses; under '||' recovery would fire
// immediately on cycle 1. Assert the derated scale is HELD for the first
// (saturation_hold_cycles - 1) on-target unsaturated cycles, then ramps.
TEST_F(DiagonalMotionCovTest, RecoveryWaitsForHoldWindow)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    setSaturated(true);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);
    const double sp_derated = m.getTelemetry().back().speed_scale;
    ASSERT_LT(sp_derated, pid_config_.saturation_recovery_threshold);

    // Clear saturation and hold on target. unsaturated_cycles counts up from 0.
    setSaturated(false);
    setWorldPose(0.0, 0.0, 1.5);

    const int hold = pid_config_.saturation_hold_cycles;  // 5
    // The first (hold - 1) cycles increment unsaturated_cycles to 1..hold-1,
    // all of which leave can_recover == false => NO change under '&&'.
    // (Telemetry records the scale at the START of each cycle, so this also
    // verifies no recovery was applied in the preceding cycles.)
    for (int i = 0; i < hold; ++i)
    {
        m.update(kDt);
        EXPECT_NEAR(m.getTelemetry().back().speed_scale, sp_derated, 1e-9)
            << "scale changed during hold window at cycle " << i;
    }
    // After the hold window elapses, recovery begins and the scale ramps up.
    // Under the '||' mutant the scale would already have risen during the hold
    // window above, failing the loop assertion.
    for (int i = 0; i < 5; ++i)
        m.update(kDt);
    EXPECT_GT(m.getTelemetry().back().speed_scale, sp_derated);
}

// L239 ' >= '->' > ': can_recover = unsaturated_cycles_ >= saturation_hold_cycles.
// This pins the EXACT cycle recovery starts. With hold=5: under '>=' recovery is
// applied at the end of the 5th unsaturated on-target update (and telemetry,
// which records the scale at the START of a cycle, shows the bump on the 6th
// update). Under '>' it needs unsaturated_cycles==6, shifting everything one
// cycle later, so after exactly 6 updates the scale would still read the floor.
TEST_F(DiagonalMotionCovTest, RecoveryStartsExactlyAtHoldBoundary)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    setSaturated(true);
    for (int i = 0; i < 60; ++i)
        m.update(kDt);
    const double floor = m.getTelemetry().back().speed_scale;  // == saturation_min_scale
    ASSERT_NEAR(floor, pid_config_.saturation_min_scale, 1e-9);

    setSaturated(false);
    setWorldPose(0.0, 0.0, 1.5);  // on target => yaw_error 0

    const int hold = pid_config_.saturation_hold_cycles;  // 5
    // First `hold` updates: telemetry must still read the floor (recovery, if
    // any, is applied at the END of the hold-th update, not yet recorded).
    for (int i = 0; i < hold; ++i)
    {
        m.update(kDt);
        EXPECT_NEAR(m.getTelemetry().back().speed_scale, floor, 1e-9);
    }
    // The (hold+1)-th update records the start-of-cycle scale AFTER the first
    // recovery step under '>='. Under '>' it would still read the floor.
    m.update(kDt);
    EXPECT_GT(m.getTelemetry().back().speed_scale, floor);
    EXPECT_NEAR(m.getTelemetry().back().speed_scale,
                floor + pid_config_.saturation_recovery_rate, 1e-9);
}

// No derate when saturated but heading error is below the saturation threshold.
TEST_F(DiagonalMotionCovTest, NoDerateWhenHeadingErrorSmall)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);  // on target => yaw_error 0
    m.start();
    setSaturated(true);           // saturated, but error below threshold
    for (int i = 0; i < 20; ++i)
        m.update(kDt);

    EXPECT_NEAR(m.getTelemetry().back().speed_scale, 1.0, 1e-9);
}

// ===================================================================
// Trapezoidal profile through the PRODUCTION path (DiagonalMotion ->
// ProfiledPIDController -> TrapezoidalProfile::calculate). The profiled
// setpoint trajectory is driven purely by the trapezoidal math (it does
// NOT depend on the measured position), so we hold the robot at start and
// pin the EXACT setpoint position/velocity per cycle. All expected values
// are derived from the WPILib TrapezoidProfile::Calculate spec documented
// in trapezoidal_profile.hpp (NOT echoed from the binary):
//
//   constraints {max_v=1, a_acc=2, a_dec=2}, goal d=2.0, dt=0.01, start {0,0}.
//   Accel: v = a*t, x = 0.5*a*t^2. At cycle n (t=n*dt):
//     n=1  -> v=0.02,  x=0.5*2*0.01^2          = 0.0001
//     n=2  -> v=0.04,  x=0.5*2*0.02^2          = 0.0004
//     n=50 -> v=1.00,  x=0.5*2*0.5^2           = 0.2500  (end of accel)
//   Cruise (v capped at max_v=1): x advances by v*dt = 0.01/cycle:
//     n=100 -> v=1.00, x=0.25 + 1.0*(0.50)     = 0.7500
//     n=150 -> v=1.00, x=0.25 + 1.0*(1.00)     = 1.2500
//   These pin the accel branch (L195/196), the velocity cap / cruise_v
//   (cruise branch L201-204), and the +/-/* arithmetic of the trapezoid
//   distance bookkeeping (L162-188).
// ===================================================================
TEST_F(DiagonalMotionCovTest, ProfileSetpointAccelThenCruiseExact)
{
    pid_config_.linear = AxisConstraints{1.0, 2.0, 2.0};  // max_v=1, a=2, dec=2
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 2.0;          // clear accel -> cruise -> decel profile
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);   // hold at start: profile still advances
    m.start();

    auto step = [&]() { m.update(kDt); return m.getTelemetry().back(); };

    auto c1 = step();
    EXPECT_NEAR(c1.setpoint_velocity_mps, 0.02, 1e-9);
    EXPECT_NEAR(c1.setpoint_position_m, 0.0001, 1e-9);

    auto c2 = step();
    EXPECT_NEAR(c2.setpoint_velocity_mps, 0.04, 1e-9);
    EXPECT_NEAR(c2.setpoint_position_m, 0.0004, 1e-9);

    // Advance to end of acceleration (cycle 50): v hits the cap, x=0.25.
    DiagonalMotionTelemetry c50{};
    for (int i = 2; i < 50; ++i) c50 = step();
    EXPECT_NEAR(c50.setpoint_velocity_mps, 1.0, 1e-9);
    EXPECT_NEAR(c50.setpoint_position_m, 0.25, 1e-9);

    // Cruise: velocity pinned at the cap, position advances linearly.
    DiagonalMotionTelemetry c100{};
    for (int i = 50; i < 100; ++i) c100 = step();
    EXPECT_NEAR(c100.setpoint_velocity_mps, 1.0, 1e-9);
    EXPECT_NEAR(c100.setpoint_position_m, 0.75, 1e-9);

    DiagonalMotionTelemetry c150{};
    for (int i = 100; i < 150; ++i) c150 = step();
    EXPECT_NEAR(c150.setpoint_velocity_mps, 1.0, 1e-9);
    EXPECT_NEAR(c150.setpoint_position_m, 1.25, 1e-9);
}

// Cruise phase with a NON-UNIT cruise velocity pins end_full_speed = end_accel +
// full_speed_dist/cruise_v (L187). With cruise_v != 1 the '/'->'*' mutant changes
// the cruise-window length, shifting the phase boundary and the cruise setpoint.
//   {max_v=0.5, a=2, dec=2}, goal d=2.0, dt=0.01:
//     accel ends at cycle 25 (v=0.5, x=0.5^2/(2*2)=0.0625),
//     cruise advances x by v*dt = 0.005/cycle:
//       c50  -> v=0.5, x=0.0625 + 0.5*0.25 = 0.1875
//       c100 -> v=0.5, x=0.0625 + 0.5*0.75 = 0.4375
TEST_F(DiagonalMotionCovTest, ProfileSetpointCruiseNonUnitVelocityExact)
{
    pid_config_.linear = AxisConstraints{0.5, 2.0, 2.0};  // cruise_v cap = 0.5 != 1
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 2.0;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    auto step = [&]() { m.update(kDt); return m.getTelemetry().back(); };

    DiagonalMotionTelemetry c25{};
    for (int i = 0; i < 25; ++i) c25 = step();
    EXPECT_NEAR(c25.setpoint_velocity_mps, 0.5, 1e-9);     // end of accel at the cap
    EXPECT_NEAR(c25.setpoint_position_m, 0.0625, 1e-9);

    DiagonalMotionTelemetry c50{};
    for (int i = 25; i < 50; ++i) c50 = step();
    EXPECT_NEAR(c50.setpoint_velocity_mps, 0.5, 1e-9);
    EXPECT_NEAR(c50.setpoint_position_m, 0.1875, 1e-9);

    DiagonalMotionTelemetry c100{};
    for (int i = 50; i < 100; ++i) c100 = step();
    EXPECT_NEAR(c100.setpoint_velocity_mps, 0.5, 1e-9);
    EXPECT_NEAR(c100.setpoint_position_m, 0.4375, 1e-9);
}

// Deceleration branch of TrapezoidalProfile::calculate (computed BACKWARD
// from the goal, L206-211). Same constraints, d=0.5 so the profile peaks at
// v=1.0 at cycle 50 (x=0.25) then decelerates symmetrically to the goal.
// Decel is the mirror of accel about the midpoint:
//   r_vel = g_vel + time_left*a_dec ; r_pos = g_pos - (g_vel + tl*a_dec*0.5)*tl
// At cycle 70 (20 cycles = 0.20 s past the v=1.0 peak), with a_dec=2:
//   v = 1.0 - 2*0.20 = 0.60 ;  x = 0.5 - 0.5*2*(remaining)... derived = 0.41
//   (verified from the backward-integration form: time_left at c70 = 0.30,
//    r_vel = 0 + 0.30*2 = 0.60, r_pos = 0.5 - (0 + 0.30*2*0.5)*0.30 = 0.41)
TEST_F(DiagonalMotionCovTest, ProfileSetpointDecelBranchExact)
{
    pid_config_.linear = AxisConstraints{1.0, 2.0, 2.0};
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.5;          // triangular peak at v=1.0, then decel
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    DiagonalMotionTelemetry last{};
    for (int i = 0; i < 70; ++i) { m.update(kDt); last = m.getTelemetry().back(); }

    // Cycle 70 lands squarely in the backward-from-goal deceleration branch.
    EXPECT_NEAR(last.setpoint_velocity_mps, 0.60, 1e-9);
    EXPECT_NEAR(last.setpoint_position_m, 0.41, 1e-9);

    // One more cycle: v drops by exactly a_dec*dt = 0.02, x advances < v*dt.
    m.update(kDt);
    EXPECT_NEAR(m.getTelemetry().back().setpoint_velocity_mps, 0.58, 1e-9);
    EXPECT_NEAR(m.getTelemetry().back().setpoint_position_m, 0.4159, 1e-9);
}

// Triangular profile with ASYMMETRIC accel/decel exercises the v_peak^2
// formula (L177-181): v_peak = sqrt(2*d*a_acc*a_dec/(a_acc+a_dec)) and the
// asymmetric accel_time = v_peak/a_acc. With {max_v=2, a_acc=8, a_dec=2} and
// d=0.05 (too short to reach max_v=2):
//   v_peak = sqrt(2*0.05*8*2/(8+2)) = sqrt(0.16) = 0.4 (never hits max_v).
//   Accel uses a_acc=8: cycle n (dt=0.001) -> v = 8*n*0.001, x = 0.5*8*(n*0.001)^2
//     n=1 -> v=0.008, x=0.000004 ; n=2 -> v=0.016, x=0.000016
//   The peak velocity reached is 0.4, well below max_v=2 (genuinely triangular).
// Pins L177/L178 a_acc*a_dec product and the (a_acc+a_dec) denominator.
TEST_F(DiagonalMotionCovTest, ProfileSetpointTriangularAsymmetricExact)
{
    pid_config_.linear = AxisConstraints{2.0, 8.0, 2.0};  // max_v=2, a_acc=8, a_dec=2
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.05;          // too short to reach max_v => triangular
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    const double kDt2 = 0.001;
    auto step = [&]() { m.update(kDt2); return m.getTelemetry().back(); };

    auto c1 = step();
    EXPECT_NEAR(c1.setpoint_velocity_mps, 0.008, 1e-9);   // a_acc=8 ramp
    EXPECT_NEAR(c1.setpoint_position_m, 0.000004, 1e-9);

    auto c2 = step();
    EXPECT_NEAR(c2.setpoint_velocity_mps, 0.016, 1e-9);
    EXPECT_NEAR(c2.setpoint_position_m, 0.000016, 1e-9);

    // Peak velocity over the run must equal the analytic v_peak = 0.4, and must
    // stay strictly below max_v=2 (a corrupted a_acc*a_dec product or +/- in the
    // denominator shifts this peak).
    double peak = std::max(c1.setpoint_velocity_mps, c2.setpoint_velocity_mps);
    for (int i = 0; i < 200; ++i) peak = std::max(peak, step().setpoint_velocity_mps);
    EXPECT_NEAR(peak, 0.4, 1e-6);
    EXPECT_LT(peak, 2.0);
}

// ===================================================================
// L94 ' > '->' >= '/'<= ': the finished-branch guard `if (dt > 0.0)` that
// pushes a zero velocity + drive update once finished. We complete the
// motion, then call update with dt>0 (must drive once: telemetry stays put
// but no crash) and with dt==0 (must NOT call drive().update). We verify
// the boundary by confirming a finished update with dt==0 is a pure no-op
// and one with dt>0 keeps the motion finished and commands zero.
// ===================================================================
TEST_F(DiagonalMotionCovTest, FinishedZeroDtDoesNotAdvance)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.10;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setWorldPose(0.10, 0.0, 0.0);
    for (int i = 0; i < 60; ++i) m.update(kDt);
    ASSERT_TRUE(m.isFinished());

    const size_t n = m.getTelemetry().size();
    m.update(0.0);   // finished + dt<=0: must be a pure no-op
    EXPECT_EQ(m.getTelemetry().size(), n);
    m.update(kDt);   // finished + dt>0: still finished, no telemetry growth
    EXPECT_TRUE(m.isFinished());
    EXPECT_EQ(m.getTelemetry().size(), n);
}

// L147 ' < '->' >= ': the settling guard requires |filtered_velocity| BELOW
// kSettlingVelocity to declare done. If the robot is AT the target distance
// (error ~0) but still moving FAST, the original must NOT complete; the '>='
// mutant would wrongly complete. We jump straight to the target in one cycle so
// position error is ~0 but the filtered velocity is large (~ d/dt).
TEST_F(DiagonalMotionCovTest, DoesNotCompleteAtTargetWhileMovingFast)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 0.30;
    cfg.has_distance_target = true;
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    // Jump to the target in a single step: error ~0 but raw vel = 0.30/0.01 = 30,
    // filtered = 0.3*30 = 9.0 >> kSettlingVelocity(0.02). Under '<' (orig) the
    // settling clause is false => NOT finished. Under '>=' it would complete.
    setWorldPose(0.30, 0.0, 0.0);
    m.update(kDt);

    ASSERT_FALSE(m.getTelemetry().empty());
    EXPECT_GT(std::abs(m.getTelemetry().back().filtered_velocity_mps),
              0.02);                    // genuinely moving fast at target
    EXPECT_NEAR(m.getTelemetry().back().actual_error_m, 0.0, 1e-4);  // at target
    EXPECT_FALSE(m.isFinished());       // must NOT have completed
}

// L162 ' - '->' ' (noop on the lower clamp bound -max_velocity_): the primary
// command is clamped to [-max_velocity_, +max_velocity_]. A REVERSE diagonal
// (negative distance target) drives the profiled command negative, so the
// LOWER bound is the binding one. Held far from a negative goal, the command
// must saturate at exactly -max_velocity_ (= -1.0 here). If the '-' is dropped
// the lower bound becomes +max_velocity_ (clamp interval collapses/inverts) and
// the negative command would not be reproduced.
TEST_F(DiagonalMotionCovTest, ReverseCommandClampedToNegativeMaxVelocity)
{
    pid_config_.linear = AxisConstraints{1.0, 2.0, 2.0};  // max_v = 1.0
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = -10.0;        // far negative goal => command saturates negative
    cfg.target_heading_rad = 0.0;
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    for (int i = 0; i < 80; ++i) m.update(kDt);

    double min_vx = 0.0;
    for (const auto& t : m.getTelemetry())
    {
        EXPECT_GE(t.cmd_vx_mps, -1.0 - 1e-9);   // never below -max_velocity_
        min_vx = std::min(min_vx, t.cmd_vx_mps);
    }
    EXPECT_NEAR(min_vx, -1.0, 1e-6);            // reaches the negative cap exactly
}

// L240 ' < '->' >= ': needs_recovery = speed_scale_ < recovery_threshold || ...
// The first clause must independently enable recovery. We derate ONLY the
// speed_scale (a single saturated cycle: 1.0 -> 0.9) WITHOUT touching the
// heading_scale (which stays 1.0, so the second clause is FALSE). Then we clear
// saturation and hold on target. Under the original '<': 0.9 < 0.95 is TRUE so
// recovery fires and speed_scale ramps up. Under the '>=' mutant: 0.9 >= 0.95 is
// FALSE and heading_scale(1.0) < 0.95 is FALSE => needs_recovery FALSE => NO
// recovery, speed_scale frozen at 0.9. We assert it ramps up by recovery_rate.
TEST_F(DiagonalMotionCovTest, RecoveryFiresWhenOnlySpeedScaleBelowThreshold)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;   // large yaw error so the derate branch arms
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();

    // Exactly ONE saturated derate cycle: speed_scale 1.0 -> 0.9, heading 1.0.
    setSaturated(true);
    m.update(kDt);
    setSaturated(false);

    // Sanity: only the speed scale was derated (single geometric step), and it
    // sits below the 0.95 recovery threshold while heading_scale is untouched.
    // Run one more (unsaturated, but still large yaw error => no recovery yet)
    // to surface the applied derate in telemetry.
    setWorldPose(0.0, 0.0, 0.0);  // heading 0, target 1.5 => large yaw error
    m.update(kDt);
    const double sp_after_derate = m.getTelemetry().back().speed_scale;
    const double hs_after_derate = m.getTelemetry().back().heading_scale;
    ASSERT_NEAR(sp_after_derate, pid_config_.saturation_derating_factor, 1e-9);  // 0.9
    ASSERT_NEAR(hs_after_derate, 1.0, 1e-9);                                     // untouched
    ASSERT_LT(sp_after_derate, pid_config_.saturation_recovery_threshold);       // < 0.95
    ASSERT_GE(hs_after_derate, pid_config_.saturation_recovery_threshold);       // >= 0.95

    // Clear + hold on target so recovery is eligible after the hold window.
    setWorldPose(0.0, 0.0, 1.5);  // on target => yaw_error 0
    for (int i = 0; i < 15; ++i)
        m.update(kDt);

    // Recovery must have raised speed_scale (driven solely by clause 1).
    EXPECT_GT(m.getTelemetry().back().speed_scale, sp_after_derate);
}

// Heading-scale derating ELSE branch (L223-227): once speed_scale has bottomed
// at saturation_min_scale, further sustained saturation derates heading_scale by
// the heading_saturation_derating_factor (0.85) per cycle. Pin the EXACT first
// heading derate step 1.0 -> 0.85 (mirror of the speed_scale 0.9 step test) so a
// corruption of heading_saturation_derating_factor's use is caught.
TEST_F(DiagonalMotionCovTest, HeadingScaleDeratesByExactFactorOnceSpeedBottomed)
{
    DiagonalMotionConfig cfg;
    cfg.angle_rad = 0.0;
    cfg.distance_m = 100.0;
    cfg.target_heading_rad = 1.5;   // sustained large yaw error
    auto m = makeDiagonal(cfg);

    setWorldPose(0.0, 0.0, 0.0);
    m.start();
    setSaturated(true);

    // Drive until speed_scale has bottomed at the floor and heading_scale has
    // taken its FIRST derate step. speed_scale: 1.0,0.9,...,->0.2 (floor) then
    // heading_scale: 1.0 -> 0.85 on the next saturated cycle.
    double first_heading_derate = -1.0;
    double prev_hs = 1.0;
    for (int i = 0; i < 60; ++i)
    {
        m.update(kDt);
        const double hs = m.getTelemetry().back().heading_scale;
        if (hs < prev_hs - 1e-9 && first_heading_derate < 0.0)
            first_heading_derate = hs / prev_hs;   // geometric step ratio
        prev_hs = hs;
    }
    ASSERT_GT(first_heading_derate, 0.0) << "heading_scale never derated";
    // The first observed heading derate step must equal the documented factor.
    EXPECT_NEAR(first_heading_derate,
                pid_config_.heading_saturation_derating_factor, 1e-9);  // 0.85
}
