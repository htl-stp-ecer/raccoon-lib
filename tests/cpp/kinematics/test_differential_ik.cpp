// Mutation-resistant tests for DifferentialKinematics forward/inverse transforms.
//
// Suite prefix: KinDiffIK  (ctest -R 'KinDiffIK')
//
// The public IK output we assert against is MotorCommands.wheel_velocities,
// which carries the raw rad/s wheel targets *before* the BEMF conversion the
// MotorAdapter applies — i.e. the exact result of the inverse-kinematics math.
// estimateState() is exercised by feeding the encoder filter through repeated
// applyCommand() calls (which differentiate getPosition()).

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "kinematics/differential/differential.hpp"
#include "foundation/speed_mode_context.hpp"

#include <cmath>

using namespace libstp::test;
using libstp::foundation::ChassisVelocity;
using libstp::foundation::MotorCalibration;
using libstp::foundation::SpeedModeContext;
using libstp::kinematics::differential::DifferentialKinematics;

namespace {

constexpr double kTol = 1e-9;
constexpr double kWheelbase = 0.20;   // 200 mm
constexpr double kRadius    = 0.05;   // 50 mm

// Guarantees SpeedMode is OFF for the duration of a test and restored after,
// so global state never leaks between tests.
class KinDiffIK : public KinematicsTestFixture {
protected:
    void SetUp() override {
        KinematicsTestFixture::SetUp();
        prev_speed_mode_ = SpeedModeContext::instance().isSpeedModeEnabled();
        SpeedModeContext::instance().setSpeedModeEnabled(false);
    }
    void TearDown() override {
        SpeedModeContext::instance().setSpeedModeEnabled(prev_speed_mode_);
    }

    DifferentialKinematics make(double wb = kWheelbase, double r = kRadius) {
        return DifferentialKinematics(motors_[0].get(), motors_[1].get(), wb, r);
    }

    bool prev_speed_mode_{false};
};

// ---------------------------------------------------------------------------
// Constructor validation
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, CtorRejectsNullLeftMotor) {
    EXPECT_THROW(DifferentialKinematics(nullptr, motors_[1].get(), kWheelbase, kRadius),
                 std::invalid_argument);
}

TEST_F(KinDiffIK, CtorRejectsNullRightMotor) {
    EXPECT_THROW(DifferentialKinematics(motors_[0].get(), nullptr, kWheelbase, kRadius),
                 std::invalid_argument);
}

TEST_F(KinDiffIK, CtorRejectsNonPositiveWheelbase) {
    EXPECT_THROW(make(0.0, kRadius), std::invalid_argument);
    EXPECT_THROW(make(-0.1, kRadius), std::invalid_argument);
}

TEST_F(KinDiffIK, CtorRejectsNonPositiveRadius) {
    EXPECT_THROW(make(kWheelbase, 0.0), std::invalid_argument);
    EXPECT_THROW(make(kWheelbase, -0.05), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Inverse kinematics: applyCommand wheel velocities
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, PureForwardEqualWheels) {
    auto k = make();
    // vx = 0.5 m/s, wz = 0  ->  both wheels = vx/R = 10 rad/s
    auto out = k.applyCommand(ChassisVelocity{0.5, 0.0, 0.0}, 0.01);
    ASSERT_EQ(out.wheel_velocities.size(), 2u);
    EXPECT_NEAR(out.wheel_velocities[0], 10.0, kTol);  // left
    EXPECT_NEAR(out.wheel_velocities[1], 10.0, kTol);  // right
    EXPECT_FALSE(out.saturated_any);
}

TEST_F(KinDiffIK, PureRotationAntisymmetricWheels) {
    auto k = make();
    // wz = +2 rad/s (CCW), vx = 0.
    // v_left  = (0 - wz*b/2)/R = -(2*0.1)/0.05 = -4 rad/s
    // v_right = (0 + wz*b/2)/R =  (2*0.1)/0.05 = +4 rad/s
    auto out = k.applyCommand(ChassisVelocity{0.0, 0.0, 2.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], -4.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], +4.0, kTol);
    // Pure rotation: left == -right, sum == 0.
    EXPECT_NEAR(out.wheel_velocities[0] + out.wheel_velocities[1], 0.0, kTol);
}

TEST_F(KinDiffIK, CombinedForwardAndRotation) {
    auto k = make();
    // vx = 0.3, wz = 1.0
    // v_left  = (0.3 - 1.0*0.1)/0.05 = 0.2/0.05 = 4
    // v_right = (0.3 + 1.0*0.1)/0.05 = 0.4/0.05 = 8
    auto out = k.applyCommand(ChassisVelocity{0.3, 0.0, 1.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], 4.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 8.0, kTol);
}

TEST_F(KinDiffIK, LateralVelocityIsIgnoredByDifferential) {
    auto k = make();
    // vy must have NO effect on a differential drive.
    auto with_vy    = k.applyCommand(ChassisVelocity{0.4, 0.7, 0.5}, 0.01);
    auto without_vy = k.applyCommand(ChassisVelocity{0.4, 0.0, 0.5}, 0.01);
    EXPECT_NEAR(with_vy.wheel_velocities[0], without_vy.wheel_velocities[0], kTol);
    EXPECT_NEAR(with_vy.wheel_velocities[1], without_vy.wheel_velocities[1], kTol);
}

TEST_F(KinDiffIK, NegativeForwardReversesBothWheels) {
    auto k = make();
    auto out = k.applyCommand(ChassisVelocity{-0.25, 0.0, 0.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], -5.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], -5.0, kTol);
}

TEST_F(KinDiffIK, ZeroCommandZeroWheels) {
    auto k = make();
    auto out = k.applyCommand(ChassisVelocity{0.0, 0.0, 0.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], 0.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 0.0, kTol);
    EXPECT_FALSE(out.saturated_any);
}

// ---------------------------------------------------------------------------
// Saturation / desaturation
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, NoSaturationWhenBelowLimit) {
    auto k = make();
    k.setMaxWheelSpeed(20.0);
    EXPECT_DOUBLE_EQ(k.getMaxWheelSpeed(), 20.0);
    auto out = k.applyCommand(ChassisVelocity{0.5, 0.0, 0.0}, 0.01);  // 10 rad/s
    EXPECT_FALSE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 10.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 10.0, kTol);
}

TEST_F(KinDiffIK, SaturationScalesBothWheelsPreservingRatio) {
    auto k = make();
    k.setMaxWheelSpeed(6.0);
    // raw: left=4, right=8 -> max_abs=8 > 6 -> scale = 6/8 = 0.75
    // left=3, right=6
    auto out = k.applyCommand(ChassisVelocity{0.3, 0.0, 1.0}, 0.01);
    EXPECT_TRUE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 3.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 6.0, kTol);
    // Ratio preserved: left/right == 4/8 == 0.5
    EXPECT_NEAR(out.wheel_velocities[0] / out.wheel_velocities[1], 0.5, kTol);
    // Dominant wheel sits exactly on the limit.
    EXPECT_NEAR(std::max(std::abs(out.wheel_velocities[0]),
                         std::abs(out.wheel_velocities[1])), 6.0, kTol);
}

TEST_F(KinDiffIK, SaturationBoundaryExactlyAtLimitDoesNotScale) {
    auto k = make();
    k.setMaxWheelSpeed(10.0);
    // Exactly 10 rad/s; condition is strict ">" so no scaling.
    auto out = k.applyCommand(ChassisVelocity{0.5, 0.0, 0.0}, 0.01);
    EXPECT_FALSE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 10.0, kTol);
}

TEST_F(KinDiffIK, MaxWheelSpeedZeroDisablesLimiting) {
    auto k = make();
    k.setMaxWheelSpeed(0.0);  // disabled
    auto out = k.applyCommand(ChassisVelocity{5.0, 0.0, 0.0}, 0.01);  // 100 rad/s
    EXPECT_FALSE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 100.0, kTol);
}

TEST_F(KinDiffIK, SaturationWithNegativeDominantWheel) {
    auto k = make();
    k.setMaxWheelSpeed(2.0);
    // wz=2 -> left=-4, right=+4; max_abs=4>2 -> scale 0.5 -> -2, +2
    auto out = k.applyCommand(ChassisVelocity{0.0, 0.0, 2.0}, 0.01);
    EXPECT_TRUE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], -2.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], +2.0, kTol);
}

// ---------------------------------------------------------------------------
// Forward kinematics: estimateState (round trip through the encoder filter)
// ---------------------------------------------------------------------------

// Helper: drive both motor encoder positions so that, with vel_lpf_alpha=1,
// the filtered velocity equals the desired wheel rad/s exactly.
TEST_F(KinDiffIK, EstimateStateForwardFromEqualWheels) {
    // alpha = 1.0 -> no filtering, w_meas_filt_ == raw differentiated velocity.
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);
    motors_[0]->simulatePosition(0);
    motors_[1]->simulatePosition(0);

    auto k = make();
    const double dt = 0.01;
    // Prime the baseline (first setVelocity -> updateEncoderVelocity records pos_prev).
    k.applyCommand(ChassisVelocity{0.0, 0.0, 0.0}, dt);

    // Advance both wheels by 8 ticks*ticks_to_rad(1.0) over dt=0.01 -> 800 rad/s.
    motors_[0]->advancePosition(8);
    motors_[1]->advancePosition(8);
    k.applyCommand(ChassisVelocity{0.0, 0.0, 0.0}, dt);

    auto est = k.estimateState();
    // both wheels = 800 rad/s -> vx = (wl+wr)*R/2 = 800*0.05 = 40, wz=0
    EXPECT_NEAR(est.vx, 40.0, 1e-6);
    EXPECT_NEAR(est.vy, 0.0, 1e-12);   // differential always reports vy=0
    EXPECT_NEAR(est.wz, 0.0, 1e-9);
}

TEST_F(KinDiffIK, EstimateStatePureRotationFromOppositeWheels) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);
    motors_[0]->simulatePosition(0);
    motors_[1]->simulatePosition(0);

    auto k = make();
    const double dt = 0.01;
    k.applyCommand(ChassisVelocity{0.0, 0.0, 0.0}, dt);

    // left -5 ticks, right +5 ticks over dt -> w_left=-500, w_right=+500
    motors_[0]->advancePosition(-5);
    motors_[1]->advancePosition(+5);
    k.applyCommand(ChassisVelocity{0.0, 0.0, 0.0}, dt);

    auto est = k.estimateState();
    // vx = (wl+wr)*R/2 = 0; wz = (wr-wl)*R/b = (500-(-500))*0.05/0.2 = 1000*0.05/0.2 = 250
    EXPECT_NEAR(est.vx, 0.0, 1e-6);
    EXPECT_NEAR(est.wz, 250.0, 1e-6);
}

// True round trip: forward(inverse(v)) == v, expressed at the encoder level.
TEST_F(KinDiffIK, RoundTripForwardInverse) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);
    motors_[0]->simulatePosition(0);
    motors_[1]->simulatePosition(0);

    auto k = make();
    const double dt = 0.01;
    const ChassisVelocity cmd{0.3, 0.0, 1.0};

    // Inverse: compute wheel rad/s targets.
    auto out = k.applyCommand(cmd, dt);  // also primes baseline
    const double wl = out.wheel_velocities[0];  // 4.0
    const double wr = out.wheel_velocities[1];  // 8.0

    // Feed those exact wheel velocities back through the encoders:
    // position delta = w * dt / ticks_to_rad = w * 0.01.
    motors_[0]->advancePosition(static_cast<int>(std::lround(wl * dt)));  // 0 (rounds) -> use scaled cal instead
    motors_[1]->advancePosition(static_cast<int>(std::lround(wr * dt)));
    k.applyCommand(cmd, dt);

    auto est = k.estimateState();
    // Because ticks are integers, wl*dt=0.04 and wr*dt=0.08 round to 0 — so use
    // the analytic check instead of the lossy integer path: drive larger ticks.
    // (Re-do with a clean integer-friendly setup below.)
    (void)est;

    // Clean integer-friendly round trip: pick wheel velocities that map to
    // whole ticks. ticks_to_rad=1, dt=0.01 -> w = ticks/dt = 100*ticks.
    // Want wl=400, wr=800 (= 4 and 8 ticks).
    motors_[0]->simulatePosition(0);
    motors_[1]->simulatePosition(0);
    k.resetEncoders();
    k.applyCommand(cmd, dt);            // re-prime
    motors_[0]->advancePosition(4);     // 4 ticks -> 400 rad/s
    motors_[1]->advancePosition(8);     // 8 ticks -> 800 rad/s
    k.applyCommand(cmd, dt);
    auto est2 = k.estimateState();
    // forward of (400, 800): vx=(400+800)*0.05/2=30; wz=(800-400)*0.05/0.2=100
    EXPECT_NEAR(est2.vx, 30.0, 1e-6);
    EXPECT_NEAR(est2.wz, 100.0, 1e-6);
    // And these are exactly the forward map of the inverse ratio: vx/wz == 30/100
    // matches cmd.vx/cmd.wz == 0.3/1.0 == 0.3. 30/100 = 0.3. Consistency check:
    EXPECT_NEAR(est2.vx / est2.wz, cmd.vx / cmd.wz, 1e-9);
}

// ---------------------------------------------------------------------------
// applyPowerCommand (open-loop PWM via setSpeed)
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, ApplyPowerForwardEqualPercent) {
    auto k = make();
    // direction pure forward -> wl=wr -> both at full power_percent.
    EXPECT_CALL(*motors_[0], setSpeed(80));
    EXPECT_CALL(*motors_[1], setSpeed(80));
    k.applyPowerCommand(ChassisVelocity{1.0, 0.0, 0.0}, 80);
}

TEST_F(KinDiffIK, ApplyPowerRotationOppositePercent) {
    auto k = make();
    // wz only: w_left=-wz*b/2, w_right=+wz*b/2 -> equal magnitude, opposite.
    // Dominant gets +/-100, scaled.
    EXPECT_CALL(*motors_[0], setSpeed(-100));
    EXPECT_CALL(*motors_[1], setSpeed(100));
    k.applyPowerCommand(ChassisVelocity{0.0, 0.0, 1.0}, 100);
}

TEST_F(KinDiffIK, ApplyPowerMixedNormalizesToDominant) {
    auto k = make();
    // direction vx=0.3, wz=1.0 -> w_left=0.3-0.1=0.2, w_right=0.3+0.1=0.4
    // max_abs=0.4, scale=100/0.4=250 -> left=round(0.2*250)=50, right=round(0.4*250)=100
    EXPECT_CALL(*motors_[0], setSpeed(50));
    EXPECT_CALL(*motors_[1], setSpeed(100));
    k.applyPowerCommand(ChassisVelocity{0.3, 0.0, 1.0}, 100);
}

// Boundary: direction whose dominant magnitude is *exactly* 1e-9. The early
// return guard is `max_abs < 1e-9`: at exactly 1e-9 it must NOT return — it
// proceeds and commands the motors. A `<=` mutant would early-return and skip
// the setSpeed calls. With vx=1e-9, scale=power/1e-9, p=round(1e-9*scale)=power.
TEST_F(KinDiffIK, ApplyPowerDominantExactlyAtEpsilonStillCommands) {
    auto k = make();
    // Pure forward direction -> w_left = w_right = direction.vx = 1e-9.
    EXPECT_CALL(*motors_[0], setSpeed(80));
    EXPECT_CALL(*motors_[1], setSpeed(80));
    k.applyPowerCommand(ChassisVelocity{1e-9, 0.0, 0.0}, 80);
}

TEST_F(KinDiffIK, ApplyPowerZeroDirectionNoCalls) {
    auto k = make();
    // max_abs < 1e-9 -> early return, no setSpeed calls.
    EXPECT_CALL(*motors_[0], setSpeed(::testing::_)).Times(0);
    EXPECT_CALL(*motors_[1], setSpeed(::testing::_)).Times(0);
    k.applyPowerCommand(ChassisVelocity{0.0, 0.0, 0.0}, 100);
}

// ---------------------------------------------------------------------------
// SpeedMode path
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, SpeedModeScalesDominantWheelTo100Percent) {
    SpeedModeContext::instance().setSpeedModeEnabled(true);
    auto k = make();
    // vx=0.3, wz=1.0 -> v_left=4, v_right=8 (rad/s). Dominant=8 -> 100, left=50.
    EXPECT_CALL(*motors_[0], setSpeed(50));
    EXPECT_CALL(*motors_[1], setSpeed(100));
    auto out = k.applyCommand(ChassisVelocity{0.3, 0.0, 1.0}, 0.01);
    // wheel_velocities still report the raw rad/s, saturated_any false in SpeedMode.
    EXPECT_NEAR(out.wheel_velocities[0], 4.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 8.0, kTol);
    EXPECT_FALSE(out.saturated_any);
}

// Boundary: a command whose dominant wheel speed is *exactly* the 1e-9
// threshold. vx = 1e-9*R gives v_left = v_right = vx/R == 1e-9 in double.
// The guard is `sm_max_abs >= 1e-9`: at exactly 1e-9 the scaling branch must
// run (pct=100), which a `>` mutant would skip (leaving pct=0).
TEST_F(KinDiffIK, SpeedModeDominantExactlyAtEpsilonScalesNotZero) {
    SpeedModeContext::instance().setSpeedModeEnabled(true);
    auto k = make();
    // sm_max_abs == 1e-9 -> scale=100/1e-9, v*scale=100 -> lround clamps to 100.
    EXPECT_CALL(*motors_[0], setSpeed(100));
    EXPECT_CALL(*motors_[1], setSpeed(100));
    k.applyCommand(ChassisVelocity{1e-9 * kRadius, 0.0, 0.0}, 0.01);
}

TEST_F(KinDiffIK, SpeedModeZeroCommandZeroPercent) {
    SpeedModeContext::instance().setSpeedModeEnabled(true);
    auto k = make();
    EXPECT_CALL(*motors_[0], setSpeed(0));
    EXPECT_CALL(*motors_[1], setSpeed(0));
    auto out = k.applyCommand(ChassisVelocity{0.0, 0.0, 0.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], 0.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 0.0, kTol);
}

// ---------------------------------------------------------------------------
// hardStop / resetEncoders / getMotors / getWheelRadius
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, HardStopBrakesBothMotors) {
    auto k = make();
    EXPECT_CALL(*motors_[0], brake());
    EXPECT_CALL(*motors_[1], brake());
    k.hardStop();
}

TEST_F(KinDiffIK, GetMotorsReturnsLeftThenRight) {
    auto k = make();
    auto motors = k.getMotors();
    ASSERT_EQ(motors.size(), 2u);
    EXPECT_EQ(motors[0], motors_[0].get());
    EXPECT_EQ(motors[1], motors_[1].get());
}

TEST_F(KinDiffIK, GetWheelRadiusReturnsCtorValue) {
    auto k = make(kWheelbase, 0.037);
    EXPECT_DOUBLE_EQ(k.getWheelRadius(), 0.037);
}

TEST_F(KinDiffIK, ResetEncodersReprimesBaselineSoNoSpuriousVelocity) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);
    motors_[0]->simulatePosition(0);
    motors_[1]->simulatePosition(0);
    auto k = make();
    const double dt = 0.01;

    // Establish a non-zero velocity estimate.
    k.applyCommand(ChassisVelocity{0, 0, 0}, dt);
    motors_[0]->advancePosition(5);
    motors_[1]->advancePosition(5);
    k.applyCommand(ChassisVelocity{0, 0, 0}, dt);
    EXPECT_GT(k.estimateState().vx, 0.0);

    // After reset, a large absolute-position jump must not be turned into
    // velocity — the next sample only re-primes the baseline.
    k.resetEncoders();
    motors_[0]->advancePosition(1000);
    motors_[1]->advancePosition(1000);
    k.applyCommand(ChassisVelocity{0, 0, 0}, dt);
    auto est = k.estimateState();
    EXPECT_NEAR(est.vx, 0.0, 1e-9);
    EXPECT_NEAR(est.wz, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// getStmOdometryConfig — matrix coefficients, port ordering, inversion sign
// ---------------------------------------------------------------------------

TEST_F(KinDiffIK, StmOdometryConfigCoefficientsAndPorts) {
    // motors_[0] is port 0 (left), motors_[1] is port 1 (right) by fixture default.
    MotorCalibration cal; cal.ticks_to_rad = 0.01; cal.bemf_offset = 3.0;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);

    auto k = make();  // wheelbase 0.2, radius 0.05
    auto cfg = k.getStmOdometryConfig();

    const float R = 0.05f;
    const float b = 0.2f;
    // inv_matrix row 0 (vx): left R/2, right R/2
    EXPECT_FLOAT_EQ(cfg.inv_matrix[0][0], R / 2.0f);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[0][1], R / 2.0f);
    // row 1 (vy) all zero
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][0], 0.0f);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][1], 0.0f);
    // row 2 (wz): left -R/b, right +R/b
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][0], -R / b);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][1], +R / b);

    // fwd_matrix: left [1/R, 0, -b/(2R)], right [1/R, 0, +b/(2R)]
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[0][0], 1.0f / R);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[0][1], 0.0f);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[0][2], -b / (2.0f * R));
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[1][0], 1.0f / R);   // right wheel 1/R term
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[1][1], 0.0f);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[1][2], +b / (2.0f * R));

    // ticks_to_rad copied per port, not inverted.
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[0], 0.01f);
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[1], 0.01f);
    EXPECT_FLOAT_EQ(cfg.bemf_offset[0], 3.0f);
    EXPECT_FLOAT_EQ(cfg.bemf_offset[1], 3.0f);
}

TEST_F(KinDiffIK, StmOdometryConfigInvertedMotorNegatesTicksToRad) {
    // Rebuild left motor as inverted (port 0), right normal (port 1).
    motors_[0] = std::make_unique<testing::NiceMock<MockMotor>>(0, /*inverted=*/true);
    MotorCalibration cal; cal.ticks_to_rad = 0.02; cal.bemf_offset = 1.5;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);

    auto k = make();
    auto cfg = k.getStmOdometryConfig();
    // Inverted left -> ticks_to_rad negated; bemf_offset is a magnitude (unsigned).
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[0], -0.02f);
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[1], +0.02f);
    EXPECT_FLOAT_EQ(cfg.bemf_offset[0], 1.5f);
}

TEST_F(KinDiffIK, StmOdometryConfigReordersByHardwarePort) {
    // Left motor on port 3, right motor on port 1 -> slot data must land in
    // those columns, not in slot order.
    motors_[0] = std::make_unique<testing::NiceMock<MockMotor>>(3, false);  // left, port 3
    motors_[1] = std::make_unique<testing::NiceMock<MockMotor>>(1, false);  // right, port 1
    MotorCalibration cal; cal.ticks_to_rad = 0.01;
    motors_[0]->setCalibration(cal);
    motors_[1]->setCalibration(cal);

    auto k = make();
    auto cfg = k.getStmOdometryConfig();
    const float R = 0.05f, b = 0.2f;
    // wz row: left(port3) = -R/b, right(port1) = +R/b
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][3], -R / b);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][1], +R / b);
    // untouched columns stay zero
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][0], 0.0f);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][2], 0.0f);
}

}  // namespace
