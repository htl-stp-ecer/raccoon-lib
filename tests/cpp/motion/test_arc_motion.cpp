#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cmath>
#include <numbers>

#include "drive/drive.hpp"
#include "motion/arc_motion.hpp"
#include "motion/motion_config.hpp"
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

// Distinctive suite prefix: ArcMotionCov
class ArcMotionCovTest : public ::testing::Test
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
        // angular: max_v=2, accel=4, decel=4 ; linear: max_v=1, accel=2, decel=2
        pid_config_.angular = AxisConstraints{2.0, 4.0, 4.0};
        pid_config_.linear = AxisConstraints{1.0, 2.0, 2.0};
    }

    ArcMotion makeArc(ArcMotionConfig cfg)
    {
        MotionContext ctx{*drive_, *mock_odometry_, pid_config_};
        return ArcMotion(ctx, cfg);
    }

    // Set absolute heading the controller reads.
    void setAbsHeading(double h)
    {
        ON_CALL(*mock_odometry_, getAbsoluteHeading()).WillByDefault(Return(h));
    }

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::shared_ptr<NiceMock<MockOdometry>> mock_odometry_;
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_;
};

// ===================================================================
// Lifecycle
// ===================================================================

TEST_F(ArcMotionCovTest, NotFinishedBeforeUpdate)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);
    EXPECT_FALSE(arc.isFinished());
}

TEST_F(ArcMotionCovTest, StartDoesNotResetOdometry)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);
    EXPECT_CALL(*mock_odometry_, reset()).Times(0);
    setAbsHeading(0.7);
    arc.start();
}

TEST_F(ArcMotionCovTest, UpdateCallsOdometryUpdate)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);
    setAbsHeading(0.0);
    EXPECT_CALL(*mock_odometry_, update(kDt)).Times(AtLeast(1));
    arc.update(kDt);
}

TEST_F(ArcMotionCovTest, NonPositiveDtIsNoOpAfterStart)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);
    setAbsHeading(0.0);
    arc.start();
    arc.update(0.0);  // dt<=0 => returns without recording telemetry
    EXPECT_TRUE(arc.getTelemetry().empty());
}

// ===================================================================
// Body-frame arc progress (absolute heading delta)
// ===================================================================

TEST_F(ArcMotionCovTest, ArcProgressIsAbsoluteHeadingDelta)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;   // 90 deg CCW
    auto arc = makeArc(cfg);

    setAbsHeading(1.0);  // absolute start heading
    arc.start();

    setAbsHeading(1.3);  // advanced +0.3 rad
    arc.update(kDt);

    const auto& t = arc.getTelemetry();
    ASSERT_FALSE(t.empty());
    EXPECT_NEAR(t.back().heading_rad, 0.3, 1e-9);
    EXPECT_NEAR(t.back().heading_error_rad, kPi / 2.0 - 0.3, 1e-9);
    // arc_position = |progress|*radius
    EXPECT_NEAR(t.back().arc_position_m, 0.3 * 0.2, 1e-9);
    EXPECT_NEAR(t.back().arc_target_m, (kPi / 2.0) * 0.2, 1e-9);
}

// ===================================================================
// Velocity derivation: vx = |omega| * radius (drive arc)
// ===================================================================

TEST_F(ArcMotionCovTest, DriveArcDerivesForwardFromOmega)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.25;
    cfg.arc_angle_rad = kPi;    // big target so it accelerates
    cfg.lateral = false;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    // Hold heading at start: the trapezoidal setpoint leads the measurement, so
    // the profiled PID commands forward progress toward the +angle goal.
    arc.update(kDt);

    const auto& t = arc.getTelemetry();
    ASSERT_FALSE(t.empty());
    const auto& s = t.back();
    EXPECT_GT(s.cmd_wz_radps, 0.0);                       // CCW => positive omega
    EXPECT_NEAR(s.cmd_vx_mps, std::abs(s.cmd_wz_radps) * 0.25, 1e-9);
    EXPECT_NEAR(s.cmd_vy_mps, 0.0, 1e-12);                // not lateral
}

// CW arc (negative angle) => negative omega, vx still positive (|omega|).
TEST_F(ArcMotionCovTest, ClockwiseArcNegativeOmegaPositiveForward)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.25;
    cfg.arc_angle_rad = -kPi;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    arc.update(kDt);  // profile leads toward negative goal => negative omega

    const auto& s = arc.getTelemetry().back();
    EXPECT_LT(s.cmd_wz_radps, 0.0);
    EXPECT_GE(s.cmd_vx_mps, 0.0);
    EXPECT_NEAR(s.cmd_vx_mps, std::abs(s.cmd_wz_radps) * 0.25, 1e-9);
}

// ===================================================================
// Reverse arc (negative speed_scale): back along the SAME circle
// ===================================================================

// A reverse LEFT arc drives backwards (vx<0) AND flips the heading goal so the
// robot turns the opposite way (omega<0). vx and omega keep the same-sign
// relationship as the forward arc, so the arc centre stays on the same side.
TEST_F(ArcMotionCovTest, ReverseLeftArcBacksAlongSameCircle)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.25;
    cfg.arc_angle_rad = kPi;     // left (CCW) arc geometry
    cfg.speed_scale = -1.0;      // negative => reverse
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    arc.update(kDt);

    const auto& s = arc.getTelemetry().back();
    EXPECT_LT(s.target_angle_rad, 0.0);            // goal negated (turns CW now)
    EXPECT_LT(s.cmd_wz_radps, 0.0);                // omega follows the negated goal
    EXPECT_LT(s.cmd_vx_mps, 0.0);                  // and the robot moves backwards
    EXPECT_NEAR(s.cmd_vx_mps, -std::abs(s.cmd_wz_radps) * 0.25, 1e-9);
    EXPECT_GT(s.cmd_vx_mps * s.cmd_wz_radps, 0.0); // same-side ICR (same circle)
}

// A negative speed of magnitude 1 is NOT clamped to a near-stall crawl: the
// derived reverse speed matches a forward speed=+1 arc, just mirrored in sign.
TEST_F(ArcMotionCovTest, ReverseSpeedMagnitudeMatchesForward)
{
    ArcMotionConfig fwd;
    fwd.radius_m = 0.25;
    fwd.arc_angle_rad = kPi;
    fwd.speed_scale = 1.0;
    auto arc_fwd = makeArc(fwd);
    setAbsHeading(0.0);
    arc_fwd.start();
    arc_fwd.update(kDt);

    ArcMotionConfig rev = fwd;
    rev.speed_scale = -1.0;
    auto arc_rev = makeArc(rev);
    setAbsHeading(0.0);
    arc_rev.start();
    arc_rev.update(kDt);

    const double vx_fwd = arc_fwd.getTelemetry().back().cmd_vx_mps;
    const double vx_rev = arc_rev.getTelemetry().back().cmd_vx_mps;
    EXPECT_GT(vx_fwd, 0.0);
    EXPECT_NEAR(vx_rev, -vx_fwd, 1e-9);            // same magnitude, opposite sign
}

// Reverse strafe (lateral) arc: vy flips together with the travel direction.
TEST_F(ArcMotionCovTest, ReverseLateralArcFlipsVy)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.25;
    cfg.arc_angle_rad = kPi;   // forward would strafe right (+vy)
    cfg.lateral = true;
    cfg.speed_scale = -1.0;    // reverse
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    arc.update(kDt);

    const auto& s = arc.getTelemetry().back();
    EXPECT_NEAR(s.cmd_vx_mps, 0.0, 1e-12);         // lateral => no vx
    EXPECT_LT(s.cmd_vy_mps, 0.0);                  // reversed from the forward +vy
    EXPECT_NEAR(s.cmd_vy_mps, -std::abs(s.cmd_wz_radps) * 0.25, 1e-9);
}

// ===================================================================
// Lateral (strafe) arc: velocity goes to vy with sign from arc direction
// ===================================================================

TEST_F(ArcMotionCovTest, LateralArcPositiveAngleStrafesRightPositiveVy)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.25;
    cfg.arc_angle_rad = kPi;   // CCW => +vy
    cfg.lateral = true;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    arc.update(kDt);

    const auto& s = arc.getTelemetry().back();
    EXPECT_NEAR(s.cmd_vx_mps, 0.0, 1e-12);     // lateral => no vx
    EXPECT_GT(s.cmd_vy_mps, 0.0);
    EXPECT_NEAR(s.cmd_vy_mps, std::abs(s.cmd_wz_radps) * 0.25, 1e-9);
}

TEST_F(ArcMotionCovTest, LateralArcNegativeAngleStrafesLeftNegativeVy)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.25;
    cfg.arc_angle_rad = -kPi;   // CW => -vy
    cfg.lateral = true;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    arc.update(kDt);

    const auto& s = arc.getTelemetry().back();
    EXPECT_NEAR(s.cmd_vx_mps, 0.0, 1e-12);
    EXPECT_LT(s.cmd_vy_mps, 0.0);
}

// ===================================================================
// Max angular velocity clamping: limited by linear.max_velocity/radius
// ===================================================================

// With radius=1.0 and linear.max_velocity=1.0, the linear limit (1.0 rad/s) is
// tighter than angular.max_velocity (2.0). Commanded omega must never exceed 1.0.
TEST_F(ArcMotionCovTest, OmegaClampedByLinearLimitOverLargeRadius)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 1.0;
    cfg.arc_angle_rad = 4.0 * kPi;  // very large, forces saturation attempt
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    // Drive several cycles; heading barely advances so PID pushes hard.
    double h = 0.0;
    for (int i = 0; i < 50; ++i)
    {
        h += 0.001;
        setAbsHeading(h);
        arc.update(kDt);
    }
    for (const auto& s : arc.getTelemetry())
    {
        EXPECT_LE(std::abs(s.cmd_wz_radps), 1.0 + 1e-9);
        EXPECT_LE(s.cmd_vx_mps, 1.0 + 1e-9);  // derived forward also under linear cap
    }
}

// With a tiny radius the angular limit (2.0) is the binding constraint.
TEST_F(ArcMotionCovTest, OmegaClampedByAngularLimitSmallRadius)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.05;
    cfg.arc_angle_rad = 4.0 * kPi;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    double h = 0.0;
    for (int i = 0; i < 50; ++i)
    {
        h += 0.001;
        setAbsHeading(h);
        arc.update(kDt);
    }
    for (const auto& s : arc.getTelemetry())
        EXPECT_LE(std::abs(s.cmd_wz_radps), 2.0 + 1e-9);
}

// speed_scale halves the effective max angular velocity.
TEST_F(ArcMotionCovTest, SpeedScaleLimitsOmega)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.05;          // angular-limited
    cfg.arc_angle_rad = 4.0 * kPi;
    cfg.speed_scale = 0.5;        // => max omega 1.0
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    double h = 0.0;
    for (int i = 0; i < 50; ++i)
    {
        h += 0.001;
        setAbsHeading(h);
        arc.update(kDt);
    }
    for (const auto& s : arc.getTelemetry())
        EXPECT_LE(std::abs(s.cmd_wz_radps), 1.0 + 1e-9);
}

// ===================================================================
// Completion + hasReachedAngle
// ===================================================================

TEST_F(ArcMotionCovTest, HasReachedAngleFalseBeforeStart)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);
    EXPECT_FALSE(arc.hasReachedAngle());
}

TEST_F(ArcMotionCovTest, HasReachedAngleTrueWhenWithinTolerance)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    // Move exactly to target.
    setAbsHeading(kPi / 2.0);
    EXPECT_TRUE(arc.hasReachedAngle());
}

TEST_F(ArcMotionCovTest, CompletesWhenAtTargetAndSettled)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    // Hold heading exactly at target so velocity filters to ~0 and error~0.
    setAbsHeading(kPi / 2.0);
    for (int i = 0; i < 50; ++i)
        arc.update(kDt);

    EXPECT_TRUE(arc.isFinished());
    EXPECT_TRUE(arc.hasReachedAngle());
}

// Once finished, further updates command zero velocity and don't advance.
TEST_F(ArcMotionCovTest, FinishedUpdateCommandsZero)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = 0.0;   // target 0 => completes immediately when settled
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    for (int i = 0; i < 5; ++i)
        arc.update(kDt);
    ASSERT_TRUE(arc.isFinished());

    const size_t n_before = arc.getTelemetry().size();
    arc.update(kDt);  // finished branch: no telemetry appended
    EXPECT_EQ(arc.getTelemetry().size(), n_before);
}

// ===================================================================
// startWarm: seeds velocity + offset without odometry reset
// ===================================================================

TEST_F(ArcMotionCovTest, StartWarmSeedsFilteredVelocity)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi;
    auto arc = makeArc(cfg);

    setAbsHeading(0.5);
    arc.startWarm(/*heading_offset_rad=*/0.0, /*initial_angular_velocity=*/1.2);
    EXPECT_NEAR(arc.getFilteredVelocity(), 1.2, 1e-9);
}

// ===================================================================
// Hardened: filtered angular velocity arithmetic (L144 /dt, L145 blend)
// ===================================================================

// Cycle 1 from rest of the filter: filtered = alpha*raw + (1-alpha)*0.
// raw = remainder(h1-h0, 2*pi)/dt. With alpha=0.3, h0=1.0, h1=1.5, dt=0.01:
//   delta=0.5 ; raw = 0.5/0.01 = 50 ; filtered = 0.3*50 = 15.0.
// The delta 0.5 > pi/10 so remainder's modulo base matters: the L144 ' * '->' / '
// mutant turns 2*pi into 2/pi (~0.637) and remainder(0.5, 0.637) = -0.137 != 0.5,
// flipping the velocity sign. Also kills L144 ' / '->' * ' (raw would be tiny) and
// L145 ' * '->' / ' (0.3/50 instead of 0.3*50).
TEST_F(ArcMotionCovTest, FilteredVelocityFirstCycleExact)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi;  // far target so it never completes here
    auto arc = makeArc(cfg);

    setAbsHeading(1.0);
    arc.start();
    setAbsHeading(1.5);       // delta = 0.5 rad
    arc.update(kDt);

    const auto& t = arc.getTelemetry().back();
    EXPECT_NEAR(t.filtered_velocity_radps, 15.0, 1e-9);
}

// Cycle 2 with zero new motion: filtered = 0.3*0 + 0.7*15.0 = 10.5.
// Kills L145 ' + '->' - ' (would give -10.5) and the (1-alpha) blend mutation.
TEST_F(ArcMotionCovTest, FilteredVelocitySecondCycleBlend)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi;
    auto arc = makeArc(cfg);

    setAbsHeading(1.0);
    arc.start();
    setAbsHeading(1.5);
    arc.update(kDt);           // filtered -> 15.0
    arc.update(kDt);           // hold 1.5 => raw 0 ; filtered -> 0.7*15.0 = 10.5

    const auto& t = arc.getTelemetry().back();
    EXPECT_NEAR(t.filtered_velocity_radps, 10.5, 1e-9);
}

// ===================================================================
// Hardened: max angular velocity derivation with radius != 1 so the
// '/' vs '*' on radius and the scale multiply are distinguishable.
// linear.max_velocity = 1.0, radius = 0.5 => linear limit = 1.0/0.5 = 2.0.
// angular.max_velocity = 2.0 => min(2.0, 2.0) = 2.0, * scale 1.0 = 2.0.
// To make the linear limit the binding constraint and pin the value, use
// radius = 0.8 => linear limit = 1.25 < angular 2.0 => omega cap = 1.25.
// Kills L22 ' / '->' * ' (1.0*0.8=0.8) and L25 ' * '->' / ' (1/1.25).
// ===================================================================
TEST_F(ArcMotionCovTest, OmegaCapEqualsLinearLimitOverRadiusNonUnit)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.8;             // linear limit = 1.0/0.8 = 1.25 rad/s
    cfg.arc_angle_rad = 4.0 * kPi;  // huge => PID saturates the cap
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    double h = 0.0;
    double peak = 0.0;
    for (int i = 0; i < 80; ++i)
    {
        h += 0.0005;            // heading barely advances => PID pushes to cap
        setAbsHeading(h);
        arc.update(kDt);
        peak = std::max(peak, std::abs(arc.getTelemetry().back().cmd_wz_radps));
    }
    // Commanded omega must reach (but never exceed) the linear-derived cap 1.25.
    EXPECT_LE(peak, 1.25 + 1e-9);
    EXPECT_NEAR(peak, 1.25, 1e-6);
    // Derived forward vx = |omega|*radius must hit ~ linear.max_velocity (1.0).
    double vpeak = 0.0;
    for (const auto& s : arc.getTelemetry())
        vpeak = std::max(vpeak, s.cmd_vx_mps);
    EXPECT_NEAR(vpeak, 1.0, 1e-6);  // 1.25 * 0.8 = 1.0
}

// speed_scale multiplies the cap: scale 0.4 => cap 0.4*1.25 = 0.5. Pins L25.
TEST_F(ArcMotionCovTest, SpeedScaleMultipliesLinearCapNonUnitRadius)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.8;
    cfg.arc_angle_rad = 4.0 * kPi;
    cfg.speed_scale = 0.4;          // cap = 0.4 * 1.25 = 0.5
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    double h = 0.0;
    double peak = 0.0;
    for (int i = 0; i < 80; ++i)
    {
        h += 0.0005;
        setAbsHeading(h);
        arc.update(kDt);
        peak = std::max(peak, std::abs(arc.getTelemetry().back().cmd_wz_radps));
    }
    EXPECT_NEAR(peak, 0.5, 1e-6);
}

// ===================================================================
// Hardened hasReachedAngle: progress is (current_abs - initial_abs).
// With a NONZERO initial heading, the '-' must be a subtraction; '+' would
// compute the wrong progress and report not-reached (or reached too early).
// ===================================================================
TEST_F(ArcMotionCovTest, HasReachedAngleUsesSubtractionFromInitialHeading)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = kPi / 2.0;   // 90 deg
    auto arc = makeArc(cfg);

    setAbsHeading(1.0);   // nonzero absolute start
    arc.start();

    // Not yet at target: progress = 1.2 - 1.0 = 0.2 rad, well short of pi/2.
    setAbsHeading(1.2);
    EXPECT_FALSE(arc.hasReachedAngle());

    // At target: progress = (1.0 + pi/2) - 1.0 = pi/2. The '+' mutant would
    // compute remainder(1.0 + (1.0+pi/2)) which is NOT ~pi/2 => stays false.
    setAbsHeading(1.0 + kPi / 2.0);
    EXPECT_TRUE(arc.hasReachedAngle());
}

// ===================================================================
// Hardened: angular acceleration is scaled by linear.acceleration / radius
// (L42). The profiled-PID setpoint velocity ramps at exactly this scaled
// accel. With radius=0.8, angular.accel=4, linear.accel=2:
//   scaled accel = min(4, 2/0.8) = min(4, 2.5) = 2.5 rad/s^2.
// Per cycle (dt=0.01) the setpoint velocity rises by 2.5*0.01 = 0.025.
// The '/'->'*' mutant gives min(4, 2*0.8) = 1.6 => 0.016 per cycle.
// ===================================================================
TEST_F(ArcMotionCovTest, AngularAccelScaledByLinearOverRadius)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.8;
    cfg.arc_angle_rad = 4.0 * kPi;  // far => profile keeps accelerating
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    // Hold measured heading at start so the profile setpoint leads cleanly and
    // its velocity ramps at the (scaled) acceleration limit.
    setAbsHeading(0.0);
    arc.update(kDt);
    EXPECT_NEAR(arc.getTelemetry().back().setpoint_velocity_radps, 0.025, 1e-9);

    setAbsHeading(0.0);
    arc.update(kDt);
    EXPECT_NEAR(arc.getTelemetry().back().setpoint_velocity_radps, 0.050, 1e-9);

    setAbsHeading(0.0);
    arc.update(kDt);
    EXPECT_NEAR(arc.getTelemetry().back().setpoint_velocity_radps, 0.075, 1e-9);
}

// ===================================================================
// Hardened: angular DECELERATION is scaled by linear.deceleration / radius
// (L43). Drive the arc by feeding the profile's own setpoint back as the
// measured heading so the profiled velocity advances cleanly accel->decel.
// In the decel phase the setpoint velocity drops by exactly decel*dt per
// cycle = 2.5 * 0.01 = 0.025 (linear.decel 2.0 / radius 0.8).
// The '/'->'*' mutant gives decel = min(4, 2*0.8) = 1.6 => 0.016 per cycle.
// ===================================================================
TEST_F(ArcMotionCovTest, AngularDecelScaledByLinearOverRadius)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.8;
    cfg.arc_angle_rad = 0.3;   // short arc => reaches decel within ~40 cycles
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    for (int i = 0; i < 40; ++i)
    {
        const double h = arc.getTelemetry().empty()
            ? 0.0 : arc.getTelemetry().back().setpoint_position_rad;
        setAbsHeading(h);
        arc.update(kDt);
    }

    const auto& tel = arc.getTelemetry();
    ASSERT_GE(tel.size(), 5u);

    // Find the first decel step (setpoint velocity decreasing) and assert the
    // per-cycle drop is exactly decel*dt = 0.025 for two consecutive steps.
    bool checked = false;
    for (size_t i = 2; i + 1 < tel.size(); ++i)
    {
        const double prev = tel[i - 1].setpoint_velocity_radps;
        const double cur = tel[i].setpoint_velocity_radps;
        const double next = tel[i + 1].setpoint_velocity_radps;
        if (cur < prev - 1e-6 && next < cur - 1e-6)  // sustained decel
        {
            EXPECT_NEAR(cur - next, 0.025, 1e-6);
            checked = true;
            break;
        }
    }
    EXPECT_TRUE(checked) << "profile never entered a sustained deceleration phase";
}

// Lateral (strafe) arc still derives vy = |omega|*radius and the omega cap is
// the linear-limited one (linear.max_velocity/radius). With radius=0.8 the omega
// cap is 1.0/0.8 = 1.25, so the lateral vy must peak at |omega|*radius = 1.0
// (== linear.max_velocity) and never exceed it. Confirms the lateral branch uses
// the same linear-derived cap as the forward branch (reviewer "missing" item).
TEST_F(ArcMotionCovTest, LateralArcVyRespectsLinearCapOverRadius)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.8;              // omega cap = 1.0/0.8 = 1.25 rad/s
    cfg.arc_angle_rad = 4.0 * kPi;   // huge => PID saturates the cap
    cfg.lateral = true;              // velocity goes to vy
    auto arc = makeArc(cfg);

    setAbsHeading(0.0);
    arc.start();
    double h = 0.0;
    double peak_vy = 0.0, peak_omega = 0.0;
    for (int i = 0; i < 80; ++i)
    {
        h += 0.0005;                 // heading barely advances => PID pushes to cap
        setAbsHeading(h);
        arc.update(kDt);
        const auto& s = arc.getTelemetry().back();
        EXPECT_NEAR(s.cmd_vx_mps, 0.0, 1e-12);                 // lateral => no vx
        EXPECT_NEAR(s.cmd_vy_mps, std::abs(s.cmd_wz_radps) * 0.8, 1e-9);  // vy = |omega|*radius
        peak_vy = std::max(peak_vy, s.cmd_vy_mps);
        peak_omega = std::max(peak_omega, std::abs(s.cmd_wz_radps));
    }
    EXPECT_LE(peak_omega, 1.25 + 1e-9);   // omega capped by linear/radius
    EXPECT_NEAR(peak_vy, 1.0, 1e-6);      // lateral velocity hits linear.max_velocity (1.25*0.8)
    EXPECT_LE(peak_vy, 1.0 + 1e-9);
}

// suppress hard stop: complete() should not throw / still mark finished.
TEST_F(ArcMotionCovTest, SuppressHardStopStillFinishes)
{
    ArcMotionConfig cfg;
    cfg.radius_m = 0.2;
    cfg.arc_angle_rad = 0.0;
    auto arc = makeArc(cfg);
    arc.setSuppressHardStopOnComplete(true);

    setAbsHeading(0.0);
    for (int i = 0; i < 5; ++i)
        arc.update(kDt);
    EXPECT_TRUE(arc.isFinished());
}
