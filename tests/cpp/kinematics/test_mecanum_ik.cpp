// Mutation-resistant tests for MecanumKinematics forward/inverse transforms.
//
// Suite prefix: KinMecIK  (ctest -R 'KinMecIK')
//
// Body-frame convention (per source): +x forward, +y right, +wz.
//   w_fl = (vx + vy - L*wz)/R
//   w_fr = (vx - vy + L*wz)/R
//   w_bl = (vx - vy - L*wz)/R
//   w_br = (vx + vy + L*wz)/R
// with L = (wheelbase + trackWidth)/2.

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "kinematics/mecanum/mecanum.hpp"
#include "foundation/speed_mode_context.hpp"

#include <cmath>

using namespace libstp::test;
using libstp::foundation::ChassisVelocity;
using libstp::foundation::MotorCalibration;
using libstp::foundation::SpeedModeContext;
using libstp::foundation::VelocityCommandGain;
using libstp::kinematics::mecanum::MecanumKinematics;

namespace {

constexpr double kTol = 1e-9;
// wheelbase 0.25, trackWidth 0.15 -> L = 0.20, R = 0.05 -> clean numbers.
constexpr double kWheelbase = 0.25;
constexpr double kTrack     = 0.15;
constexpr double kRadius    = 0.05;
constexpr double kL         = (kWheelbase + kTrack) / 2.0;  // 0.20

class KinMecIK : public KinematicsTestFixture {
protected:
    void SetUp() override {
        KinematicsTestFixture::SetUp();
        prev_speed_mode_ = SpeedModeContext::instance().isSpeedModeEnabled();
        SpeedModeContext::instance().setSpeedModeEnabled(false);
    }
    void TearDown() override {
        SpeedModeContext::instance().setSpeedModeEnabled(prev_speed_mode_);
    }

    MecanumKinematics make(VelocityCommandGain gain = {}) {
        return MecanumKinematics(motors_[0].get(), motors_[1].get(),
                                 motors_[2].get(), motors_[3].get(),
                                 kWheelbase, kTrack, kRadius, gain);
    }

    bool prev_speed_mode_{false};
};

// ---------------------------------------------------------------------------
// Constructor validation
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, CtorRejectsAnyNullMotor) {
    EXPECT_THROW(MecanumKinematics(nullptr, motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   kWheelbase, kTrack, kRadius), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), nullptr, motors_[2].get(), motors_[3].get(),
                                   kWheelbase, kTrack, kRadius), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), nullptr, motors_[3].get(),
                                   kWheelbase, kTrack, kRadius), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), nullptr,
                                   kWheelbase, kTrack, kRadius), std::invalid_argument);
}

TEST_F(KinMecIK, CtorRejectsNonPositiveGeometry) {
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   0.0, kTrack, kRadius), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   kWheelbase, 0.0, kRadius), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   kWheelbase, kTrack, -0.01), std::invalid_argument);
    // Exactly zero must also be rejected (boundary of the <= 0 guard).
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   kWheelbase, kTrack, 0.0), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   kWheelbase, 0.0, kRadius), std::invalid_argument);
    EXPECT_THROW(MecanumKinematics(motors_[0].get(), motors_[1].get(), motors_[2].get(), motors_[3].get(),
                                   0.0, kTrack, kRadius), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Inverse kinematics: applyCommand wheel velocities (fl, fr, bl, br)
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, PureForwardAllWheelsEqual) {
    auto k = make();
    // vx=0.5 -> all = 0.5/0.05 = 10 rad/s
    auto out = k.applyCommand(ChassisVelocity{0.5, 0.0, 0.0}, 0.01);
    ASSERT_EQ(out.wheel_velocities.size(), 4u);
    for (int i = 0; i < 4; ++i) EXPECT_NEAR(out.wheel_velocities[i], 10.0, kTol);
    EXPECT_FALSE(out.saturated_any);
}

TEST_F(KinMecIK, PureStrafeRightWheelPattern) {
    auto k = make();
    // vy=+0.4 (right): fl=+vy/R, fr=-vy/R, bl=-vy/R, br=+vy/R = +8,-8,-8,+8
    auto out = k.applyCommand(ChassisVelocity{0.0, 0.4, 0.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], +8.0, kTol);  // fl
    EXPECT_NEAR(out.wheel_velocities[1], -8.0, kTol);  // fr
    EXPECT_NEAR(out.wheel_velocities[2], -8.0, kTol);  // bl
    EXPECT_NEAR(out.wheel_velocities[3], +8.0, kTol);  // br
    // Strafe: net forward contribution zero.
    EXPECT_NEAR(out.wheel_velocities[0] + out.wheel_velocities[1]
              + out.wheel_velocities[2] + out.wheel_velocities[3], 0.0, kTol);
}

TEST_F(KinMecIK, PureRotationWheelPattern) {
    auto k = make();
    // wz=+1: fl=-L/R, fr=+L/R, bl=-L/R, br=+L/R = -4,+4,-4,+4
    auto out = k.applyCommand(ChassisVelocity{0.0, 0.0, 1.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], -4.0, kTol);  // fl
    EXPECT_NEAR(out.wheel_velocities[1], +4.0, kTol);  // fr
    EXPECT_NEAR(out.wheel_velocities[2], -4.0, kTol);  // bl
    EXPECT_NEAR(out.wheel_velocities[3], +4.0, kTol);  // br
    // Left side both negative, right side both positive (CCW about center).
    EXPECT_LT(out.wheel_velocities[0], 0.0);
    EXPECT_LT(out.wheel_velocities[2], 0.0);
    EXPECT_GT(out.wheel_velocities[1], 0.0);
    EXPECT_GT(out.wheel_velocities[3], 0.0);
}

TEST_F(KinMecIK, CombinedVxVyWz) {
    auto k = make();
    // vx=0.3, vy=0.2, wz=0.5; R=0.05, L=0.2
    // fl=(0.3+0.2-0.2*0.5)/0.05=(0.4)/0.05=8
    // fr=(0.3-0.2+0.2*0.5)/0.05=(0.2)/0.05=4
    // bl=(0.3-0.2-0.2*0.5)/0.05=(0.0)/0.05=0
    // br=(0.3+0.2+0.2*0.5)/0.05=(0.6)/0.05=12
    auto out = k.applyCommand(ChassisVelocity{0.3, 0.2, 0.5}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], 8.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 4.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[2], 0.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[3], 12.0, kTol);
}

TEST_F(KinMecIK, NegativeStrafeReversesPattern) {
    auto k = make();
    auto out = k.applyCommand(ChassisVelocity{0.0, -0.4, 0.0}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[0], -8.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], +8.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[2], +8.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[3], -8.0, kTol);
}

// ---------------------------------------------------------------------------
// Saturation
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, SaturationScalesAllFourPreservingRatio) {
    auto k = make();
    k.setMaxWheelSpeed(6.0);
    EXPECT_DOUBLE_EQ(k.getMaxWheelSpeed(), 6.0);
    // raw fl=8,fr=4,bl=0,br=12 -> max_abs=12 > 6 -> scale=0.5
    auto out = k.applyCommand(ChassisVelocity{0.3, 0.2, 0.5}, 0.01);
    EXPECT_TRUE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 4.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[1], 2.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[2], 0.0, kTol);
    EXPECT_NEAR(out.wheel_velocities[3], 6.0, kTol);  // dominant exactly at limit
}

TEST_F(KinMecIK, BoundaryExactlyAtLimitNoScale) {
    auto k = make();
    k.setMaxWheelSpeed(10.0);
    auto out = k.applyCommand(ChassisVelocity{0.5, 0.0, 0.0}, 0.01);  // all 10
    EXPECT_FALSE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 10.0, kTol);
}

TEST_F(KinMecIK, MaxWheelSpeedZeroDisablesLimiting) {
    auto k = make();
    k.setMaxWheelSpeed(0.0);
    auto out = k.applyCommand(ChassisVelocity{5.0, 0.0, 0.0}, 0.01);  // 100
    EXPECT_FALSE(out.saturated_any);
    EXPECT_NEAR(out.wheel_velocities[0], 100.0, kTol);
}

// ---------------------------------------------------------------------------
// Forward kinematics: estimateState
// ---------------------------------------------------------------------------

void driveAllEncoders(KinematicsTestFixture* fix_motors,
                      std::unique_ptr<testing::NiceMock<MockMotor>>* motors,
                      MecanumKinematics& k,
                      int fl, int fr, int bl, int br, double dt) {
    // Prime baseline, then advance, then sample again.
    k.applyCommand(ChassisVelocity{0, 0, 0}, dt);
    motors[0]->advancePosition(fl);
    motors[1]->advancePosition(fr);
    motors[2]->advancePosition(bl);
    motors[3]->advancePosition(br);
    k.applyCommand(ChassisVelocity{0, 0, 0}, dt);
}

TEST_F(KinMecIK, EstimateStatePureForward) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    for (int i = 0; i < 4; ++i) { motors_[i]->setCalibration(cal); motors_[i]->simulatePosition(0); }
    auto k = make();
    const double dt = 0.01;
    // all wheels +5 ticks -> all 500 rad/s
    driveAllEncoders(this, motors_, k, 5, 5, 5, 5, dt);
    auto est = k.estimateState();
    // vx = (sum)*R/4 = (2000)*0.05/4 = 25; vy=0; wz=0
    EXPECT_NEAR(est.vx, 25.0, 1e-6);
    EXPECT_NEAR(est.vy, 0.0, 1e-6);
    EXPECT_NEAR(est.wz, 0.0, 1e-6);
}

TEST_F(KinMecIK, EstimateStatePureStrafe) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    for (int i = 0; i < 4; ++i) { motors_[i]->setCalibration(cal); motors_[i]->simulatePosition(0); }
    auto k = make();
    const double dt = 0.01;
    // strafe-right pattern fl+,fr-,bl-,br+ -> +5,-5,-5,+5 ticks -> +500,-500,-500,+500
    driveAllEncoders(this, motors_, k, 5, -5, -5, 5, dt);
    auto est = k.estimateState();
    // vy = (fl - fr - bl + br)*R/4 = (500+500+500+500)*0.05/4 = 2000*0.05/4 = 25
    EXPECT_NEAR(est.vx, 0.0, 1e-6);
    EXPECT_NEAR(est.vy, 25.0, 1e-6);
    EXPECT_NEAR(est.wz, 0.0, 1e-6);
}

TEST_F(KinMecIK, EstimateStatePureRotation) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    for (int i = 0; i < 4; ++i) { motors_[i]->setCalibration(cal); motors_[i]->simulatePosition(0); }
    auto k = make();
    const double dt = 0.01;
    // rotation pattern fl-,fr+,bl-,br+ -> -5,+5,-5,+5 ticks -> -500,+500,-500,+500
    driveAllEncoders(this, motors_, k, -5, 5, -5, 5, dt);
    auto est = k.estimateState();
    // wz = (-fl + fr - bl + br)*R/(4L) = (500+500+500+500)*0.05/(4*0.2)
    //    = 2000*0.05/0.8 = 100/0.8 = 125
    EXPECT_NEAR(est.vx, 0.0, 1e-6);
    EXPECT_NEAR(est.vy, 0.0, 1e-6);
    EXPECT_NEAR(est.wz, 125.0, 1e-6);
}

// Round trip: forward(inverse(v)) == v scaled. Pick a command whose wheel
// rad/s land on whole ticks (ticks_to_rad=1, dt=0.01 -> w = ticks*100).
TEST_F(KinMecIK, RoundTripForwardInverse) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    for (int i = 0; i < 4; ++i) { motors_[i]->setCalibration(cal); motors_[i]->simulatePosition(0); }
    auto k = make();
    const double dt = 0.01;
    // Choose vx=4, vy=2, wz=5 (large, in rad/s-friendly land):
    // fl=(4+2-0.2*5)/0.05=(5)/0.05=100 -> 1 tick
    // fr=(4-2+1)/0.05=(3)/0.05=60 -> 0.6 ticks (not integer) — instead drive
    // wheels directly and confirm forward map equals the analytic inverse.
    // Use wheel rad/s of 700,500,300,900 (ticks 7,5,3,9):
    driveAllEncoders(this, motors_, k, 7, 5, 3, 9, dt);
    auto est = k.estimateState();
    // wheels: fl=700, fr=500, bl=300, br=900
    // vx=(700+500+300+900)*0.05/4 = 2400*0.05/4 = 30
    // vy=(700-500-300+900)*0.05/4 = 800*0.05/4 = 10
    // wz=(-700+500-300+900)*0.05/(4*0.2)=400*0.05/0.8=20/0.8=25
    EXPECT_NEAR(est.vx, 30.0, 1e-6);
    EXPECT_NEAR(est.vy, 10.0, 1e-6);
    EXPECT_NEAR(est.wz, 25.0, 1e-6);

    // Now push that estimated velocity back through inverse kinematics and
    // confirm we recover exactly the wheel rad/s we started from.
    auto out = k.applyCommand(ChassisVelocity{est.vx, est.vy, est.wz}, dt);
    EXPECT_NEAR(out.wheel_velocities[0], 700.0, 1e-6);
    EXPECT_NEAR(out.wheel_velocities[1], 500.0, 1e-6);
    EXPECT_NEAR(out.wheel_velocities[2], 300.0, 1e-6);
    EXPECT_NEAR(out.wheel_velocities[3], 900.0, 1e-6);
}

// ---------------------------------------------------------------------------
// applyPowerCommand
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, ApplyPowerForwardAllEqual) {
    auto k = make();
    for (int i = 0; i < 4; ++i) EXPECT_CALL(*motors_[i], setSpeed(75));
    k.applyPowerCommand(ChassisVelocity{1.0, 0.0, 0.0}, 75);
}

TEST_F(KinMecIK, ApplyPowerStrafeNormalizesToDominant) {
    auto k = make();
    // vy=1: w_fl=+1, w_fr=-1, w_bl=-1, w_br=+1 (R cancels). max_abs=1, scale=100.
    EXPECT_CALL(*motors_[0], setSpeed(100));
    EXPECT_CALL(*motors_[1], setSpeed(-100));
    EXPECT_CALL(*motors_[2], setSpeed(-100));
    EXPECT_CALL(*motors_[3], setSpeed(100));
    k.applyPowerCommand(ChassisVelocity{0.0, 1.0, 0.0}, 100);
}

TEST_F(KinMecIK, ApplyPowerMixedRounding) {
    auto k = make();
    // direction vx=0.3, vy=0.2, wz=0.5; L=0.2 (R cancels in power command):
    // w_fl=0.3+0.2-0.2*0.5=0.4; w_fr=0.3-0.2+0.1=0.2; w_bl=0.3-0.2-0.1=0.0; w_br=0.3+0.2+0.1=0.6
    // max_abs=0.6 scale=100/0.6=166.667 -> fl=round(66.67)=67, fr=round(33.33)=33, bl=0, br=100
    EXPECT_CALL(*motors_[0], setSpeed(67));
    EXPECT_CALL(*motors_[1], setSpeed(33));
    EXPECT_CALL(*motors_[2], setSpeed(0));
    EXPECT_CALL(*motors_[3], setSpeed(100));
    k.applyPowerCommand(ChassisVelocity{0.3, 0.2, 0.5}, 100);
}

TEST_F(KinMecIK, ApplyPowerZeroDirectionNoCalls) {
    auto k = make();
    for (int i = 0; i < 4; ++i)
        EXPECT_CALL(*motors_[i], setSpeed(::testing::_)).Times(0);
    k.applyPowerCommand(ChassisVelocity{0.0, 0.0, 0.0}, 100);
}

// ---------------------------------------------------------------------------
// SpeedMode path
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, SpeedModeScalesDominantTo100) {
    SpeedModeContext::instance().setSpeedModeEnabled(true);
    auto k = make();
    // vx=0.3,vy=0.2,wz=0.5 -> rad/s fl=8,fr=4,bl=0,br=12. Dominant 12 -> 100.
    // pct = round(w*100/12): fl=67, fr=33, bl=0, br=100
    EXPECT_CALL(*motors_[0], setSpeed(67));
    EXPECT_CALL(*motors_[1], setSpeed(33));
    EXPECT_CALL(*motors_[2], setSpeed(0));
    EXPECT_CALL(*motors_[3], setSpeed(100));
    auto out = k.applyCommand(ChassisVelocity{0.3, 0.2, 0.5}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[3], 12.0, kTol);
    EXPECT_FALSE(out.saturated_any);
}

TEST_F(KinMecIK, SpeedModeAllWheelsNonZeroExactPercents) {
    SpeedModeContext::instance().setSpeedModeEnabled(true);
    auto k = make();
    // Choose a command where every wheel is non-zero so the per-wheel scaling
    // is independently asserted (kills a w_bl*scale mutation).
    // vx=0.4, vy=0.1, wz=0.5, R=0.05, L=0.2:
    // fl=(0.4+0.1-0.1)/0.05=0.4/0.05=8
    // fr=(0.4-0.1+0.1)/0.05=0.4/0.05=8
    // bl=(0.4-0.1-0.1)/0.05=0.2/0.05=4
    // br=(0.4+0.1+0.1)/0.05=0.6/0.05=12
    // dominant=12 -> scale=100/12: fl=round(66.67)=67, fr=67, bl=round(33.33)=33, br=100
    EXPECT_CALL(*motors_[0], setSpeed(67));
    EXPECT_CALL(*motors_[1], setSpeed(67));
    EXPECT_CALL(*motors_[2], setSpeed(33));
    EXPECT_CALL(*motors_[3], setSpeed(100));
    auto out = k.applyCommand(ChassisVelocity{0.4, 0.1, 0.5}, 0.01);
    EXPECT_NEAR(out.wheel_velocities[2], 4.0, kTol);   // bl non-zero
    EXPECT_NEAR(out.wheel_velocities[3], 12.0, kTol);
}

// Boundary: dominant wheel speed *exactly* at the 1e-9 SpeedMode threshold.
// Pure forward vx = 1e-9*R -> every wheel = vx/R == 1e-9 in double, so
// sm_max_abs == 1e-9. The guard `sm_max_abs >= 1e-9` must take the scaling
// branch (pct=100); a `>` mutant would leave all percentages at 0.
TEST_F(KinMecIK, SpeedModeDominantExactlyAtEpsilonScalesNotZero) {
    SpeedModeContext::instance().setSpeedModeEnabled(true);
    auto k = make();
    for (int i = 0; i < 4; ++i) EXPECT_CALL(*motors_[i], setSpeed(100));
    k.applyCommand(ChassisVelocity{1e-9 * kRadius, 0.0, 0.0}, 0.01);
}

// Boundary: applyPowerCommand direction whose dominant magnitude is *exactly*
// 1e-9. The guard `max_abs < 1e-9` must NOT early-return at exactly 1e-9 — it
// proceeds and commands the motors. A `<=` mutant would skip the setSpeed
// calls. Pure forward vx=1e-9 -> all w = 1e-9; scale=power/1e-9; p=round(power).
TEST_F(KinMecIK, ApplyPowerDominantExactlyAtEpsilonStillCommands) {
    auto k = make();
    for (int i = 0; i < 4; ++i) EXPECT_CALL(*motors_[i], setSpeed(70));
    k.applyPowerCommand(ChassisVelocity{1e-9, 0.0, 0.0}, 70);
}

// ---------------------------------------------------------------------------
// hardStop / getMotors / lateral / wheelRadius
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, HardStopBrakesAllFour) {
    auto k = make();
    for (int i = 0; i < 4; ++i) EXPECT_CALL(*motors_[i], brake());
    k.hardStop();
}

TEST_F(KinMecIK, GetMotorsOrderFlFrBlBr) {
    auto k = make();
    auto motors = k.getMotors();
    ASSERT_EQ(motors.size(), 4u);
    for (int i = 0; i < 4; ++i) EXPECT_EQ(motors[i], motors_[i].get());
}

TEST_F(KinMecIK, ResetEncodersReprimesBaselineSoNoSpuriousVelocity) {
    MotorCalibration cal; cal.ticks_to_rad = 1.0; cal.vel_lpf_alpha = 1.0;
    for (int i = 0; i < 4; ++i) { motors_[i]->setCalibration(cal); motors_[i]->simulatePosition(0); }
    auto k = make();
    const double dt = 0.01;

    // Build up a non-zero velocity estimate.
    driveAllEncoders(this, motors_, k, 5, 5, 5, 5, dt);
    EXPECT_GT(k.estimateState().vx, 0.0);

    // Reset clears the filter AND drops the position baseline. A large jump in
    // absolute encoder position right after reset must NOT be differentiated
    // into a velocity (the first post-reset sample only re-primes the baseline).
    k.resetEncoders();
    for (int i = 0; i < 4; ++i) motors_[i]->advancePosition(1000);  // huge jump
    k.applyCommand(ChassisVelocity{0, 0, 0}, dt);  // re-primes baseline, no velocity
    auto est = k.estimateState();
    EXPECT_NEAR(est.vx, 0.0, 1e-9);
    EXPECT_NEAR(est.vy, 0.0, 1e-9);
    EXPECT_NEAR(est.wz, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Velocity command gains
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, DefaultVelocityGainsAreIdentity) {
    auto k = make();
    auto g = k.getVelocityCommandGains();
    EXPECT_DOUBLE_EQ(g[0], 1.0);
    EXPECT_DOUBLE_EQ(g[1], 1.0);
    EXPECT_DOUBLE_EQ(g[2], 1.0);
}

TEST_F(KinMecIK, SetVelocityGainsRoundTrip) {
    auto k = make();
    k.setVelocityCommandGains(1.1, 0.9, 1.25);
    auto g = k.getVelocityCommandGains();
    EXPECT_DOUBLE_EQ(g[0], 1.1);
    EXPECT_DOUBLE_EQ(g[1], 0.9);
    EXPECT_DOUBLE_EQ(g[2], 1.25);
}

TEST_F(KinMecIK, CtorGainFromVelocityCommandGain) {
    auto k = make(VelocityCommandGain{1.2, 0.8, 1.0});
    auto g = k.getVelocityCommandGains();
    EXPECT_DOUBLE_EQ(g[0], 1.2);
    EXPECT_DOUBLE_EQ(g[1], 0.8);
    EXPECT_DOUBLE_EQ(g[2], 1.0);
}

// ---------------------------------------------------------------------------
// getStmOdometryConfig
// ---------------------------------------------------------------------------

TEST_F(KinMecIK, StmOdometryConfigInvMatrixCoefficients) {
    MotorCalibration cal; cal.ticks_to_rad = 0.01; cal.bemf_offset = 2.0;
    for (int i = 0; i < 4; ++i) motors_[i]->setCalibration(cal);
    auto k = make();  // R=0.05, L=0.2
    auto cfg = k.getStmOdometryConfig();
    const float R = 0.05f, L = 0.2f;
    // Ports are 0,1,2,3 by fixture default (slot==port).
    // vx row: all +R/4
    for (int p = 0; p < 4; ++p) EXPECT_FLOAT_EQ(cfg.inv_matrix[0][p], R / 4.0f);
    // vy row: FL +R/4, FR -R/4, BL -R/4, BR +R/4
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][0], +R / 4.0f);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][1], -R / 4.0f);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][2], -R / 4.0f);
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][3], +R / 4.0f);
    // wz row: FL -R/(4L), FR +R/(4L), BL -R/(4L), BR +R/(4L)
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][0], -R / (4.0f * L));
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][1], +R / (4.0f * L));
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][2], -R / (4.0f * L));
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][3], +R / (4.0f * L));
}

TEST_F(KinMecIK, StmOdometryConfigFwdMatrixWithGains) {
    MotorCalibration cal; cal.ticks_to_rad = 0.01;
    for (int i = 0; i < 4; ++i) motors_[i]->setCalibration(cal);
    // Non-identity gains fold into fwd_matrix columns.
    auto k = make(VelocityCommandGain{2.0, 3.0, 4.0});
    auto cfg = k.getStmOdometryConfig();
    const float R = 0.05f, L = 0.2f;
    // FL fwd row base [1/R, +1/R, -L/R], times gains [2,3,4]:
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[0][0], (1.0f / R) * 2.0f);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[0][1], (+1.0f / R) * 3.0f);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[0][2], (-L / R) * 4.0f);
    // FR base [1/R, -1/R, +L/R]:
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[1][0], (1.0f / R) * 2.0f);   // FR 1/R term
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[1][1], (-1.0f / R) * 3.0f);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[1][2], (+L / R) * 4.0f);
    // BL base [1/R, -1/R, -L/R]:
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[2][0], (1.0f / R) * 2.0f);   // BL 1/R term
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[2][1], (-1.0f / R) * 3.0f);  // BL -1/R term
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[2][2], (-L / R) * 4.0f);
    // BR base [1/R, +1/R, +L/R]:
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[3][0], (1.0f / R) * 2.0f);   // BR 1/R term
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[3][1], (+1.0f / R) * 3.0f);
    EXPECT_FLOAT_EQ(cfg.fwd_matrix[3][2], (+L / R) * 4.0f);
}

TEST_F(KinMecIK, StmOdometryConfigInvertedMotorNegatesTicksToRad) {
    // FR (slot 1) inverted; bemf_offset stays unsigned.
    motors_[1] = std::make_unique<testing::NiceMock<MockMotor>>(1, /*inverted=*/true);
    MotorCalibration cal; cal.ticks_to_rad = 0.02; cal.bemf_offset = 4.0;
    for (int i = 0; i < 4; ++i) motors_[i]->setCalibration(cal);
    auto k = make();
    auto cfg = k.getStmOdometryConfig();
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[0], +0.02f);
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[1], -0.02f);  // inverted
    EXPECT_FLOAT_EQ(cfg.ticks_to_rad[2], +0.02f);
    EXPECT_FLOAT_EQ(cfg.bemf_offset[1], 4.0f);     // magnitude, not negated
}

TEST_F(KinMecIK, StmOdometryConfigReordersByHardwarePort) {
    // Scramble ports: FL->2, FR->0, BL->3, BR->1.
    motors_[0] = std::make_unique<testing::NiceMock<MockMotor>>(2, false);  // FL
    motors_[1] = std::make_unique<testing::NiceMock<MockMotor>>(0, false);  // FR
    motors_[2] = std::make_unique<testing::NiceMock<MockMotor>>(3, false);  // BL
    motors_[3] = std::make_unique<testing::NiceMock<MockMotor>>(1, false);  // BR
    MotorCalibration cal; cal.ticks_to_rad = 0.01;
    for (int i = 0; i < 4; ++i) motors_[i]->setCalibration(cal);
    auto k = make();
    auto cfg = k.getStmOdometryConfig();
    const float R = 0.05f, L = 0.2f;
    // vy row by slot mapped to its port:
    // FL(+R/4) -> port2, FR(-R/4) -> port0, BL(-R/4) -> port3, BR(+R/4) -> port1
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][2], +R / 4.0f);  // FL
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][0], -R / 4.0f);  // FR
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][3], -R / 4.0f);  // BL
    EXPECT_FLOAT_EQ(cfg.inv_matrix[1][1], +R / 4.0f);  // BR
    // wz row spot check on FR -> port0
    EXPECT_FLOAT_EQ(cfg.inv_matrix[2][0], +R / (4.0f * L));  // FR +R/(4L) at port0
}

}  // namespace
