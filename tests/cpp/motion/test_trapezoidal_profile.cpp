#include <gtest/gtest.h>

#include <cmath>

#include "motion/trapezoidal_profile.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::test;
using libstp::motion::TrapezoidalProfile;

// Distinctive suite prefix: TrapProfile
class TrapProfileTest : public AlgorithmTestFixture {};

using State = TrapezoidalProfile::State;
using Constraints = TrapezoidalProfile::Constraints;

// ===================================================================
// Constraints helpers + validation
// ===================================================================

TEST_F(TrapProfileTest, EffectiveDecelerationFallsBackToAccel)
{
    Constraints c;
    c.max_acceleration = 3.0;
    c.max_deceleration = 0.0;  // 0 => use accel
    EXPECT_DOUBLE_EQ(c.effectiveDeceleration(), 3.0);

    c.max_deceleration = 1.5;
    EXPECT_DOUBLE_EQ(c.effectiveDeceleration(), 1.5);
}

TEST_F(TrapProfileTest, ValidateRejectsNonPositiveMaxVelocity)
{
    Constraints c;
    c.max_velocity = 0.0;
    c.max_acceleration = 2.0;
    EXPECT_THROW(c.validate(), std::invalid_argument);
    c.max_velocity = -1.0;
    EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST_F(TrapProfileTest, ValidateRejectsNonPositiveMaxAccel)
{
    Constraints c;
    c.max_velocity = 1.0;
    c.max_acceleration = 0.0;
    EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST_F(TrapProfileTest, ValidateRejectsNegativeMaxDecel)
{
    Constraints c;
    c.max_velocity = 1.0;
    c.max_acceleration = 2.0;
    c.max_deceleration = -0.5;
    EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST_F(TrapProfileTest, ValidateAcceptsValidConstraints)
{
    Constraints c{1.0, 2.0, 0.0};
    EXPECT_NO_THROW(c.validate());
    Constraints c2{1.0, 2.0, 1.5};
    EXPECT_NO_THROW(c2.validate());
}

TEST_F(TrapProfileTest, ConstructorThrowsOnInvalidConstraints)
{
    Constraints bad;
    bad.max_velocity = -1.0;
    bad.max_acceleration = 2.0;
    EXPECT_THROW(TrapezoidalProfile(State{0.0, 0.0}, 1.0, bad), std::invalid_argument);
}

// ===================================================================
// Instance profile: trapezoidal vs triangular discrimination
// ===================================================================

// Symmetric a=2, v=1. Distance to reach full speed (accel+decel) is
// 2 * (v^2 / 2a) = v^2/a = 0.5 m. A 2 m move is clearly trapezoidal.
TEST_F(TrapProfileTest, LongMoveIsTrapezoidalWithCruisePhase)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 2.0, c);

    // accel_time = v/a = 0.5 s; decel_time = 0.5 s
    // accel_dist = decel_dist = 0.5*2*0.25 = 0.25; cruise_dist = 2 - 0.5 = 1.5
    // cruise_time = 1.5 / 1.0 = 1.5; total = 0.5 + 1.5 + 0.5 = 2.5 s
    EXPECT_NEAR(p.getTotalTime(), 2.5, 1e-9);

    // Peak velocity reaches the cruise cap exactly (1.0) mid-flight.
    const auto mid = p.getSetpoint(1.25);
    EXPECT_NEAR(mid.velocity, 1.0, 1e-9);
}

// A very short move (0.1 m) cannot reach full speed => triangular.
TEST_F(TrapProfileTest, ShortMoveIsTriangularNoCruise)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 0.1, c);

    // v_peak = sqrt(2 * d * a*a/(a+a)) = sqrt(2*0.1* (4/4)) = sqrt(0.2) ~ 0.4472
    const double v_peak = std::sqrt(0.2);
    EXPECT_LT(v_peak, 1.0);  // never reaches the cruise cap

    // total = 2 * v_peak / a = 2*0.4472/2 = 0.4472
    EXPECT_NEAR(p.getTotalTime(), 2.0 * v_peak / 2.0, 1e-9);

    // The peak velocity sample (at half total time) equals v_peak.
    const auto peak = p.getSetpoint(p.getTotalTime() / 2.0);
    EXPECT_NEAR(peak.velocity, v_peak, 1e-6);
}

// ===================================================================
// Instance profile: phase boundary position + velocity (trapezoid)
// ===================================================================

TEST_F(TrapProfileTest, PhaseBoundaryPositionsAndVelocities)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 2.0, c);

    // Start
    auto s0 = p.getSetpoint(0.0);
    EXPECT_NEAR(s0.position, 0.0, 1e-9);
    EXPECT_NEAR(s0.velocity, 0.0, 1e-9);

    // End of acceleration (t=0.5): v=1, x = 0.5*a*t^2 = 0.25
    auto accel_end = p.getSetpoint(0.5);
    EXPECT_NEAR(accel_end.velocity, 1.0, 1e-9);
    EXPECT_NEAR(accel_end.position, 0.25, 1e-9);

    // End of cruise (t=2.0): v=1, x = 0.25 + 1.0*1.5 = 1.75
    auto cruise_end = p.getSetpoint(2.0);
    EXPECT_NEAR(cruise_end.velocity, 1.0, 1e-9);
    EXPECT_NEAR(cruise_end.position, 1.75, 1e-9);

    // End (t>=total): exactly target, zero velocity
    auto done = p.getSetpoint(2.5);
    EXPECT_NEAR(done.position, 2.0, 1e-9);
    EXPECT_NEAR(done.velocity, 0.0, 1e-9);
    EXPECT_TRUE(p.isComplete(2.5));
    EXPECT_FALSE(p.isComplete(2.49));

    // Just past the start of decel (t=2.25): v should be down to 0.5
    // v = cruise - a_dec*(t-2.0) = 1.0 - 2.0*0.25 = 0.5
    auto dec = p.getSetpoint(2.25);
    EXPECT_NEAR(dec.velocity, 0.5, 1e-9);
}

// Negative target: profile mirrors, velocities are negative.
TEST_F(TrapProfileTest, NegativeTargetMirrorsSign)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, -2.0, c);

    EXPECT_NEAR(p.getTotalTime(), 2.5, 1e-9);

    auto accel_end = p.getSetpoint(0.5);
    EXPECT_NEAR(accel_end.velocity, -1.0, 1e-9);
    EXPECT_NEAR(accel_end.position, -0.25, 1e-9);

    auto done = p.getSetpoint(2.5);
    EXPECT_NEAR(done.position, -2.0, 1e-9);
}

// Exact deceleration-phase position + velocity (kills decel-arithmetic mutants).
TEST_F(TrapProfileTest, DecelPhaseExactPositionAndVelocity)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 2.0, c);
    // Decel starts at t=2.0 from position 1.75 at velocity 1.0; a_dec=2.
    // At decel_t=0.3 (t=2.3):
    //   v = 1.0 - 2.0*0.3 = 0.4
    //   x = 1.75 + 1.0*0.3 - 0.5*2.0*0.09 = 1.75 + 0.3 - 0.09 = 1.96
    auto s = p.getSetpoint(2.3);
    EXPECT_NEAR(s.velocity, 0.4, 1e-9);
    EXPECT_NEAR(s.position, 1.96, 1e-9);

    // At decel_t=0.4 (t=2.4): v = 0.2, x = 1.75 + 0.4 - 0.16 = 1.99
    auto s2 = p.getSetpoint(2.4);
    EXPECT_NEAR(s2.velocity, 0.2, 1e-9);
    EXPECT_NEAR(s2.position, 1.99, 1e-9);
}

// Nonzero initial velocity in the instance accel phase (kills the
// initial_.velocity * t term mutant at line 95).
TEST_F(TrapProfileTest, NonzeroInitialVelocityInAccelPhase)
{
    Constraints c{2.0, 2.0, 2.0};
    // Start already moving at v0=0.4 toward a far target.
    TrapezoidalProfile p(State{0.0, 0.4}, 5.0, c);
    // Accel phase: at t=0.1, accel=+2:
    //   v = 0.4 + 2*0.1 = 0.6
    //   x = 0 + 0.4*0.1 + 0.5*2*0.01 = 0.04 + 0.01 = 0.05
    auto s = p.getSetpoint(0.1);
    EXPECT_NEAR(s.velocity, 0.6, 1e-9);
    EXPECT_NEAR(s.position, 0.05, 1e-9);
}

// Asymmetric accel/decel: total time and peak differ.
TEST_F(TrapProfileTest, AsymmetricAccelDecel)
{
    // a_acc=4, a_dec=1, v=1, d=2
    Constraints c{1.0, 4.0, 1.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 2.0, c);

    // accel_time = 1/4 = 0.25; decel_time = 1/1 = 1.0
    // accel_dist = 0.5*4*0.0625 = 0.125; decel_dist = 0.5*1*1 = 0.5
    // cruise_dist = 2 - 0.625 = 1.375 > 0 -> trapezoidal
    // cruise_time = 1.375; total = 0.25 + 1.375 + 1.0 = 2.625
    EXPECT_NEAR(p.getTotalTime(), 2.625, 1e-9);

    // End of accel at t=0.25
    auto ae = p.getSetpoint(0.25);
    EXPECT_NEAR(ae.velocity, 1.0, 1e-9);
    EXPECT_NEAR(ae.position, 0.125, 1e-9);
}

// BUG (documented, not fixed): a zero-distance instance profile divides
// 0/0 in compute_profile (cruise_time_ = cruise_distance/cruise_velocity_ with
// both 0), yielding a NaN total time. isComplete() then never returns true.
// This locks in the current behavior so a future fix is detected. See notes.
// A zero-distance profile is undefined (cruise_time = 0/0 = NaN, never
// completes). Construction must reject it at config time rather than build a
// profile that can never finish.
TEST_F(TrapProfileTest, ZeroDistanceMoveIsRejectedAtConstruction)
{
    Constraints c{1.0, 2.0, 2.0};
    EXPECT_THROW(TrapezoidalProfile(State{0.0, 0.0}, 0.0, c), std::invalid_argument);
    // A nonzero target from the same initial position is still fine.
    EXPECT_NO_THROW(TrapezoidalProfile(State{0.0, 0.0}, 0.5, c));
}

// A genuinely tiny (but nonzero) move stays well-defined and triangular.
TEST_F(TrapProfileTest, VeryShortMoveIsWellDefined)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 1e-4, c);
    EXPECT_GT(p.getTotalTime(), 0.0);
    EXPECT_FALSE(std::isnan(p.getTotalTime()));
    auto done = p.getSetpoint(p.getTotalTime());
    EXPECT_NEAR(done.position, 1e-4, 1e-9);
    EXPECT_NEAR(done.velocity, 0.0, 1e-9);
}

// Negative elapsed time clamps to 0 (start state).
TEST_F(TrapProfileTest, NegativeElapsedTimeClampsToStart)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 2.0, c);
    auto s = p.getSetpoint(-5.0);
    EXPECT_NEAR(s.position, 0.0, 1e-12);
    EXPECT_NEAR(s.velocity, 0.0, 1e-12);
}

// Nonzero initial position offsets the whole profile.
TEST_F(TrapProfileTest, NonzeroInitialPosition)
{
    Constraints c{1.0, 2.0, 2.0};
    TrapezoidalProfile p(State{10.0, 0.0}, 12.0, c);  // travel +2 from x=10
    auto done = p.getSetpoint(p.getTotalTime());
    EXPECT_NEAR(done.position, 12.0, 1e-9);
    auto ae = p.getSetpoint(0.5);
    EXPECT_NEAR(ae.position, 10.25, 1e-9);
}

// ===================================================================
// Static calculate(): WPILib-style incremental profile
// ===================================================================

TEST_F(TrapProfileTest, CalculateValidatesConstraints)
{
    Constraints bad{0.0, 2.0, 2.0};
    EXPECT_THROW(
        TrapezoidalProfile::calculate(0.01, State{0.0, 0.0}, State{1.0, 0.0}, bad),
        std::invalid_argument);
}

// From rest, first step accelerates: v = a*dt, x = 0.5*a*dt^2.
TEST_F(TrapProfileTest, CalculateFirstStepAccelerates)
{
    Constraints c{1.0, 2.0, 2.0};
    const double dt = 0.01;
    auto s = TrapezoidalProfile::calculate(dt, State{0.0, 0.0}, State{2.0, 0.0}, c);
    EXPECT_NEAR(s.velocity, 2.0 * dt, 1e-12);
    EXPECT_NEAR(s.position, 0.5 * 2.0 * dt * dt, 1e-12);
}

// When already cruising at max_v far from goal, velocity stays clamped at max_v.
TEST_F(TrapProfileTest, CalculateCruisesAtMaxVelocity)
{
    Constraints c{1.0, 2.0, 2.0};
    const double dt = 0.01;
    auto s = TrapezoidalProfile::calculate(dt, State{0.0, 1.0}, State{10.0, 0.0}, c);
    EXPECT_NEAR(s.velocity, 1.0, 1e-9);
    // position advances by ~v*dt
    EXPECT_NEAR(s.position, 1.0 * dt, 1e-6);
}

// Velocity above the cap is clamped before integrating.
TEST_F(TrapProfileTest, CalculateClampsOverspeedCurrent)
{
    Constraints c{1.0, 2.0, 2.0};
    const double dt = 0.01;
    auto s = TrapezoidalProfile::calculate(dt, State{0.0, 5.0}, State{10.0, 0.0}, c);
    // current vel clamped to 1.0; in cruise it stays at 1.0
    EXPECT_NEAR(s.velocity, 1.0, 1e-9);
}

// Reverse direction: goal behind current => negative velocity command.
TEST_F(TrapProfileTest, CalculateNegativeDirection)
{
    Constraints c{1.0, 2.0, 2.0};
    const double dt = 0.01;
    auto s = TrapezoidalProfile::calculate(dt, State{0.0, 0.0}, State{-2.0, 0.0}, c);
    EXPECT_NEAR(s.velocity, -2.0 * dt, 1e-12);
    EXPECT_LT(s.position, 0.0);
}

// Goal already reached within the step => returns goal exactly.
TEST_F(TrapProfileTest, CalculateAtGoalReturnsGoal)
{
    Constraints c{1.0, 2.0, 2.0};
    auto s = TrapezoidalProfile::calculate(0.01, State{2.0, 0.0}, State{2.0, 0.0}, c);
    EXPECT_NEAR(s.position, 2.0, 1e-9);
    EXPECT_NEAR(s.velocity, 0.0, 1e-9);
}

// Large dt that overshoots the whole remaining profile => snaps to goal.
TEST_F(TrapProfileTest, CalculateLargeDtSnapsToGoal)
{
    Constraints c{1.0, 2.0, 2.0};
    auto s = TrapezoidalProfile::calculate(100.0, State{0.0, 0.0}, State{0.05, 0.0}, c);
    EXPECT_NEAR(s.position, 0.05, 1e-9);
    EXPECT_NEAR(s.velocity, 0.0, 1e-9);
}

// Integrating calculate() to convergence reaches the goal monotonically and stops.
TEST_F(TrapProfileTest, CalculateConvergesToGoal)
{
    Constraints c{1.0, 2.0, 2.0};
    const double dt = 0.01;
    State s{0.0, 0.0};
    const State goal{1.0, 0.0};
    double prev_pos = -1.0;
    for (int i = 0; i < 1000; ++i)
    {
        s = TrapezoidalProfile::calculate(dt, s, goal, c);
        EXPECT_GE(s.position, prev_pos - 1e-9);  // monotone non-decreasing
        prev_pos = s.position;
        EXPECT_LE(s.velocity, 1.0 + 1e-6);       // never exceeds cap
    }
    EXPECT_NEAR(s.position, 1.0, 1e-3);
    EXPECT_NEAR(s.velocity, 0.0, 1e-3);
}

// Asymmetric decel in calculate(): a gentler decel should still converge w/o overshoot.
TEST_F(TrapProfileTest, CalculateAsymmetricConvergesNoOvershoot)
{
    Constraints c{1.0, 4.0, 1.0};  // fast accel, slow decel
    const double dt = 0.01;
    State s{0.0, 0.0};
    const State goal{0.5, 0.0};
    for (int i = 0; i < 2000; ++i)
        s = TrapezoidalProfile::calculate(dt, s, goal, c);
    EXPECT_NEAR(s.position, 0.5, 1e-3);
    EXPECT_LT(s.position, 0.5 + 1e-2);  // no meaningful overshoot
}

// ===================================================================
// Hardened: decel-phase sign multiplication (instance profile)
// Uses cruise_velocity_ != 1 so 'sign * cruise_velocity_' differs from
// 'sign / cruise_velocity_' (kills L109/L110 ' * '->' / ' mutants), and a
// NEGATIVE target so the sign factor is genuinely -1 (not 1).
// ===================================================================
TEST_F(TrapProfileTest, DecelPhaseSignTimesVelocityNegativeTarget)
{
    // max_velocity 0.5 (cruise cap), a=2. Long move => trapezoidal, cruise=0.5.
    Constraints c{0.5, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, -2.0, c);

    // accel_time = v/a = 0.25; accel_dist = 0.5*2*0.0625 = 0.0625
    // decel_dist = 0.0625; cruise_dist = 2 - 0.125 = 1.875; cruise_time = 1.875/0.5=3.75
    // total = 0.25 + 3.75 + 0.25 = 4.25
    EXPECT_NEAR(p.getTotalTime(), 4.25, 1e-9);

    // decel_start_position_ = -(accel_dist + cruise_dist) = -(0.0625+1.875) = -1.9375
    // Decel starts at t = accel_time + cruise_time = 0.25 + 3.75 = 4.0.
    // At decel_t = 0.1 (t = 4.1), sign=-1, cruise_velocity_=0.5, a_dec=2, decel = +2:
    //   v = sign*cruise + decel*decel_t = -0.5 + 2*0.1 = -0.3
    //   x = decel_start + sign*cruise*decel_t + 0.5*decel*decel_t^2
    //     = -1.9375 + (-0.5)*0.1 + 0.5*2*0.01 = -1.9375 - 0.05 + 0.01 = -1.9775
    auto s = p.getSetpoint(4.1);
    EXPECT_NEAR(s.velocity, -0.3, 1e-9);
    EXPECT_NEAR(s.position, -1.9775, 1e-9);
}

// Positive target, cruise_velocity_ != 1, decel phase: pins the magnitude of
// 'sign * cruise_velocity_' (would be 1/0.5 = 2 under '/', not 0.5).
TEST_F(TrapProfileTest, DecelPhaseSignTimesVelocityPositiveTargetNonUnitCruise)
{
    Constraints c{0.5, 2.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 2.0, c);
    EXPECT_NEAR(p.getTotalTime(), 4.25, 1e-9);

    // decel_start_position_ = +1.9375, decel starts at t=4.0, sign=+1, decel=-2.
    // At t=4.1: v = 0.5 - 2*0.1 = 0.3 ; x = 1.9375 + 0.5*0.1 - 0.5*2*0.01 = 1.9775
    auto s = p.getSetpoint(4.1);
    EXPECT_NEAR(s.velocity, 0.3, 1e-9);
    EXPECT_NEAR(s.position, 1.9775, 1e-9);
}

// ===================================================================
// Hardened calculate(): NONZERO current velocity exercises cutoff_begin /
// cutoff_dist_begin (L157) — the "trim before current velocity" term.
// ===================================================================
TEST_F(TrapProfileTest, CalculateNonzeroCurrentVelocityAccelTerm)
{
    // a_acc=2, a_dec=2, max_v=1. Current already moving at 0.4 toward far goal.
    Constraints c{1.0, 2.0, 2.0};
    const double dt = 0.01;
    auto s = TrapezoidalProfile::calculate(dt, State{0.0, 0.4}, State{10.0, 0.0}, c);
    // Still accelerating: r_vel = c_vel + dt*a_acc = 0.4 + 0.02 = 0.42
    //   r_pos = c_pos + (c_vel + dt*a_acc*0.5)*dt = 0 + (0.4 + 0.01)*0.01 = 0.0041
    EXPECT_NEAR(s.velocity, 0.42, 1e-12);
    EXPECT_NEAR(s.position, 0.0041, 1e-12);
}

// ===================================================================
// Hardened calculate(): NONZERO goal velocity exercises cutoff_end (L159),
// cutoff_dist_end (L160), the '+ cutoff_dist_end' sum (L163), and the
// end_decel timing (L188).  We integrate to convergence and require the
// profile to settle at the goal's nonzero terminal velocity.
// ===================================================================
TEST_F(TrapProfileTest, CalculateConvergesToNonzeroGoalVelocity)
{
    Constraints c{2.0, 2.0, 2.0};
    const double dt = 0.005;
    State s{0.0, 0.0};
    const State goal{1.0, 0.5};  // arrive at x=1 still moving at 0.5 m/s
    for (int i = 0; i < 2000; ++i)
    {
        s = TrapezoidalProfile::calculate(dt, s, goal, c);
        if (s.position >= goal.position) break;
    }
    EXPECT_NEAR(s.position, 1.0, 5e-3);
    EXPECT_NEAR(s.velocity, 0.5, 5e-3);
}

// Deceleration phase with NONZERO goal velocity — EXACT single-step values.
// Current cruising at max_v=1 at x=0.97, goal x=1.0 with terminal velocity 0.4,
// a_dec=2. With dt=0.02 the step lands cleanly in the backward-from-goal decel
// branch. Hand-verified against the WPILib reconstruction (see notes):
//   cutoff_end = 0.4/2 = 0.2, cutoff_dist_end = 0.2*0.2*2*0.5 = 0.04
//   end_decel timing places dt inside decel; result pos=0.9616, vel=0.56.
// This KILLS L160 (cutoff_end*cutoff_end -> /), L163 (+cutoff_dist_end -> -),
// and L188 (end_full_speed + decel_time -> -), all of which shift the goal-vel
// terms and produce a different (pos,vel).
TEST_F(TrapProfileTest, CalculateDecelTowardNonzeroGoalVelocityExact)
{
    Constraints c{1.0, 2.0, 2.0};  // max_v=1, a_dec=2
    const double dt = 0.02;
    auto s = TrapezoidalProfile::calculate(dt, State{0.97, 1.0}, State{1.0, 0.4}, c);
    EXPECT_NEAR(s.position, 0.9616, 1e-9);
    EXPECT_NEAR(s.velocity, 0.56, 1e-9);
}

// L187 ' / '->' * ': end_full_speed = end_accel + full_speed_dist/cruise_v sets
// the cruise/decel boundary. Use cruise_v=0.5 (max_v=0.5, so /cruise_v != *cruise_v)
// and a large dt that lands MID-CRUISE far from the goal. Under '/', the cruise
// window is long and the call returns a cruise setpoint (vel=0.5, pos=2.4375);
// under '*' the window collapses and the call would report "done" at the goal.
TEST_F(TrapProfileTest, CalculateLargeDtMidCruiseStaysCruising)
{
    Constraints c{0.5, 2.0, 2.0};  // cruise_v = 0.5 != 1
    auto s = TrapezoidalProfile::calculate(5.0, State{0.0, 0.0}, State{5.0, 0.0}, c);
    EXPECT_NEAR(s.velocity, 0.5, 1e-9);     // still cruising, not snapped to goal
    EXPECT_NEAR(s.position, 2.4375, 1e-9);
}

// L141 ' > '->' >= ': direction selection when current.position == goal.position
// but the robot is still MOVING. Orig picks dir=+1 (since '>' is false), mutant
// picks dir=-1 which flips the directed frame and yields a different setpoint.
TEST_F(TrapProfileTest, CalculateDirectionAtEqualPositionMoving)
{
    Constraints c{1.0, 2.0, 2.0};
    // current x == goal x == 1.0, but current still moving forward at 0.3.
    auto s = TrapezoidalProfile::calculate(0.01, State{1.0, 0.3}, State{1.0, 0.0}, c);
    EXPECT_NEAR(s.position, 0.997282251, 1e-6);
    EXPECT_NEAR(s.velocity, 0.104264069, 1e-6);
}

// Integrate calculate() to a nonzero terminal goal velocity and confirm it
// settles there (not at zero). If cutoff_end/cutoff_dist_end (g_vel terms)
// are corrupted the terminal velocity drifts.
TEST_F(TrapProfileTest, CalculateSettlesAtNonzeroGoalVelocity)
{
    Constraints c{2.0, 2.0, 2.0};
    const double dt = 0.005;
    State s{0.0, 0.0};
    const State goal{1.0, 0.6};
    for (int i = 0; i < 4000; ++i)
    {
        s = TrapezoidalProfile::calculate(dt, s, goal, c);
        if (s.position >= goal.position) break;
    }
    EXPECT_NEAR(s.position, 1.0, 1e-2);
    EXPECT_NEAR(s.velocity, 0.6, 1e-2);
}

// ===================================================================
// Hardened calculate(): TRIANGULAR with ASYMMETRIC accel/decel pins the
// v_peak^2 = 2*d*a_acc*a_dec/(a_acc+a_dec) formula (L178 ' * '->' / ').
// A short move that cannot reach max_v, with a_acc != a_dec, gives a peak
// velocity that differs if the a_acc*a_dec product is corrupted.
// ===================================================================
TEST_F(TrapProfileTest, CalculateTriangularAsymmetricPeakVelocity)
{
    // a_acc=8, a_dec=2, max_v=2. Short move d=0.05 can't reach max_v.
    Constraints c{2.0, 8.0, 2.0};
    const double dt = 0.001;
    State s{0.0, 0.0};
    const State goal{0.05, 0.0};
    double peak = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        s = TrapezoidalProfile::calculate(dt, s, goal, c);
        peak = std::max(peak, s.velocity);
        if (s.position >= goal.position - 1e-9 && s.velocity < 1e-6 && i > 5) break;
    }
    // v_peak = sqrt(2*0.05*8*2/(8+2)) = sqrt(0.16) = 0.4
    EXPECT_NEAR(peak, 0.4, 2e-2);
    EXPECT_LT(peak, 2.0);  // genuinely triangular (never hit max_v)
    EXPECT_NEAR(s.position, 0.05, 1e-3);
}

// The instance profile triangular branch peak also pins L273/L274 region:
// asymmetric triangular peak should match the analytic v_peak.
TEST_F(TrapProfileTest, InstanceTriangularAsymmetricPeak)
{
    Constraints c{2.0, 8.0, 2.0};
    TrapezoidalProfile p(State{0.0, 0.0}, 0.05, c);
    // v_peak = sqrt(2*0.05*8*2/10) = 0.4
    const double v_peak = std::sqrt(2.0 * 0.05 * 8.0 * 2.0 / (8.0 + 2.0));
    EXPECT_NEAR(v_peak, 0.4, 1e-12);
    EXPECT_LT(v_peak, 2.0);  // triangular
    // accel_time = v_peak/a_acc = 0.05; the peak is reached at end of accel.
    auto ae = p.getSetpoint(0.4 / 8.0);
    EXPECT_NEAR(ae.velocity, 0.4, 1e-9);
}
