#include <gtest/gtest.h>
#include <cmath>
#include "foundation/pid.hpp"
#include "test_support/test_fixtures.hpp"

using libstp::foundation::PidConfig;
using libstp::foundation::PidController;
using libstp::test::AlgorithmTestFixture;

namespace
{
    // Helper: a config with only the fields we care about set. Output limits
    // are opened wide so saturation does not interfere with term arithmetic
    // unless a test explicitly tightens them.
    PidConfig makeConfig(double kp, double ki, double kd)
    {
        PidConfig c;
        c.kp = kp;
        c.ki = ki;
        c.kd = kd;
        c.output_min = -1e9;
        c.output_max = 1e9;
        return c;
    }
}

class PidEdgeTest : public AlgorithmTestFixture {};

// ---------------------------------------------------------------------------
// dt guard: zero / negative dt returns 0 and mutates no state
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, ZeroDtReturnsZeroAndNoStateChange)
{
    PidController pid(makeConfig(2.0, 1.0, 0.5));
    EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0), 0.0);
    // Integral and derivative must remain untouched.
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
}

TEST_F(PidEdgeTest, NegativeDtReturnsZeroAndNoStateChange)
{
    PidController pid(makeConfig(2.0, 1.0, 0.5));
    EXPECT_DOUBLE_EQ(pid.update(5.0, -0.01), 0.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
}

TEST_F(PidEdgeTest, ZeroDtDoesNotConsumeFirstUpdateFlag)
{
    // After a rejected zero-dt call, the first real update must still skip
    // the derivative (first_update_ stayed true).
    PidController pid(makeConfig(0.0, 0.0, 1.0)); // pure D
    pid.update(10.0, 0.0);                        // rejected
    const double out = pid.update(10.0, 0.1);     // first *valid* update
    EXPECT_DOUBLE_EQ(out, 0.0);                    // derivative skipped
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
}

// ---------------------------------------------------------------------------
// Proportional-only baseline
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, ProportionalOnly)
{
    PidController pid(makeConfig(2.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.update(3.0, 0.1), 6.0);
    EXPECT_DOUBLE_EQ(pid.update(-1.5, 0.1), -3.0);
}

// ---------------------------------------------------------------------------
// First-update derivative is skipped
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, FirstUpdateSkipsDerivative)
{
    PidController pid(makeConfig(0.0, 0.0, 5.0)); // pure D
    // Large error change, but on the very first update derivative=0.
    EXPECT_DOUBLE_EQ(pid.update(100.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
}

TEST_F(PidEdgeTest, SecondUpdateProducesDerivative)
{
    PidConfig c = makeConfig(0.0, 0.0, 1.0);
    c.derivative_lpf_alpha = 1.0; // no filtering -> raw derivative
    PidController pid(c);
    pid.update(0.0, 0.1);            // first: prev_error_=0, derivative skipped
    // error goes 0 -> 2 over dt=0.1 => raw derivative = 20.
    const double out = pid.update(2.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 20.0);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 20.0);
}

// ---------------------------------------------------------------------------
// Derivative low-pass alpha clamping
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, AlphaAboveOneClampedToOne)
{
    PidConfig c = makeConfig(0.0, 0.0, 1.0);
    c.derivative_lpf_alpha = 5.0; // out of range -> clamped to 1.0 (no filter)
    PidController pid(c);
    pid.update(0.0, 0.1);
    // raw derivative = (4-0)/0.1 = 40; alpha=1 => filtered == raw.
    EXPECT_DOUBLE_EQ(pid.update(4.0, 0.1), 40.0);
}

TEST_F(PidEdgeTest, AlphaBelowZeroClampedToZero)
{
    PidConfig c = makeConfig(0.0, 0.0, 1.0);
    c.derivative_lpf_alpha = -3.0; // clamped to 0.0 -> ignores new sample
    PidController pid(c);
    pid.update(0.0, 0.1);
    // alpha=0 => filtered_derivative_ stays at its prior value (0) forever.
    EXPECT_DOUBLE_EQ(pid.update(4.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(pid.update(8.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
}

TEST_F(PidEdgeTest, AlphaHalfBlendsNewAndOld)
{
    PidConfig c = makeConfig(0.0, 0.0, 1.0);
    c.derivative_lpf_alpha = 0.5;
    PidController pid(c);
    pid.update(0.0, 0.1);             // prev_error_ = 0
    // step 1: raw = (10-0)/0.1 = 100; filtered = 0.5*100 + 0.5*0 = 50.
    EXPECT_DOUBLE_EQ(pid.update(10.0, 0.1), 50.0);
    // step 2: error constant -> raw = 0; filtered = 0.5*0 + 0.5*50 = 25.
    EXPECT_DOUBLE_EQ(pid.update(10.0, 0.1), 25.0);
}

// ---------------------------------------------------------------------------
// Integral accumulation, deadband, and anti-windup clamp
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, IntegralAccumulates)
{
    PidController pid(makeConfig(0.0, 2.0, 0.0)); // pure I
    pid.update(1.0, 0.5); // integral = 0.5
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.5);
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.5), 2.0 * 1.0); // integral now 1.0
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 1.0);
}

TEST_F(PidEdgeTest, IntegralDeadbandBlocksAccumulation)
{
    PidConfig c = makeConfig(0.0, 1.0, 0.0);
    c.integral_deadband = 0.1; // errors with |e| <= 0.1 ignored
    PidController pid(c);
    pid.update(0.05, 1.0); // below deadband -> no accumulation
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    pid.update(0.1, 1.0);  // exactly at deadband (strict >) -> still ignored
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    pid.update(0.2, 1.0);  // above deadband -> accumulates
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.2);
}

TEST_F(PidEdgeTest, AntiWindupClampsPositive)
{
    PidConfig c = makeConfig(0.0, 1.0, 0.0);
    c.integral_max = 3.0;
    c.output_max = 1000.0; // keep output saturation out of the way
    PidController pid(c);
    // Accumulate well past the clamp.
    for (int i = 0; i < 20; ++i) pid.update(5.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 3.0); // clamped to +integral_max
}

TEST_F(PidEdgeTest, AntiWindupClampsNegative)
{
    PidConfig c = makeConfig(0.0, 1.0, 0.0);
    c.integral_max = 3.0;
    c.output_min = -1000.0;
    PidController pid(c);
    for (int i = 0; i < 20; ++i) pid.update(-5.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), -3.0); // clamped to -integral_max
}

TEST_F(PidEdgeTest, IntegralUnwindsAfterClamp)
{
    PidConfig c = makeConfig(0.0, 1.0, 0.0);
    c.integral_max = 3.0;
    c.output_min = -1000.0;
    c.output_max = 1000.0;
    PidController pid(c);
    for (int i = 0; i < 20; ++i) pid.update(5.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 3.0);
    // Reversing the error must let the integrator unwind (not stuck at clamp).
    pid.update(-1.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 2.0);
}

// ---------------------------------------------------------------------------
// Output saturation
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, OutputSaturatesHigh)
{
    PidConfig c = makeConfig(100.0, 0.0, 0.0);
    c.output_max = 7.0;
    PidController pid(c);
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.1), 7.0); // 100 clamped to 7
}

TEST_F(PidEdgeTest, OutputSaturatesLow)
{
    PidConfig c = makeConfig(100.0, 0.0, 0.0);
    c.output_min = -7.0;
    PidController pid(c);
    EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.1), -7.0); // -100 clamped to -7
}

// ---------------------------------------------------------------------------
// Derivative-on-measurement (separate deriv_signal)
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, SeparateDerivativeSignal)
{
    PidConfig c = makeConfig(1.0, 0.0, 1.0);
    c.derivative_lpf_alpha = 1.0; // raw derivative
    PidController pid(c);
    // First valid update primes prev with deriv_signal=0.
    pid.update(2.0, 0.1, 0.0);
    // P uses error (3.0 -> 3.0); D uses deriv_signal slope (0 -> 5)/0.1 = 50.
    const double out = pid.update(3.0, 0.1, 5.0);
    EXPECT_DOUBLE_EQ(out, 3.0 + 50.0);
}

// ---------------------------------------------------------------------------
// reset() clears all state including first_update_
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, ResetClearsState)
{
    PidConfig c = makeConfig(0.0, 1.0, 1.0);
    c.derivative_lpf_alpha = 1.0;
    PidController pid(c);
    pid.update(1.0, 0.1);
    pid.update(5.0, 0.1);
    EXPECT_GT(pid.getIntegral(), 0.0);
    EXPECT_NE(pid.getDerivative(), 0.0);

    pid.reset();
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
    // first_update_ must be true again: next update produces no derivative term
    // (only the integral contributes after a reset).
    pid.update(9.0, 0.1);
    EXPECT_DOUBLE_EQ(pid.getDerivative(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.9);
}

// ---------------------------------------------------------------------------
// setGains replaces only kp/ki/kd, preserving accumulated state
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, SetGainsChangesGainsKeepsIntegral)
{
    PidController pid(makeConfig(0.0, 1.0, 0.0));
    pid.update(2.0, 1.0); // integral = 2.0
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 2.0);

    pid.setGains(0.0, 3.0, 0.0); // ki: 1 -> 3, integral preserved
    // No new accumulation if error stays; i_term = 3 * (2 + 2) after next step.
    const double out = pid.update(2.0, 1.0); // integral now 4.0
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 4.0);
    EXPECT_DOUBLE_EQ(out, 3.0 * 4.0);
}

// ---------------------------------------------------------------------------
// Default config (proportional-only, kp=1) behaves as documented
// ---------------------------------------------------------------------------
TEST_F(PidEdgeTest, DefaultConfigIsProportionalUnity)
{
    PidController pid; // default config: kp=1, ki=0, kd=0
    // Output equals the error (kp=1), and ki=0 means the integrator does not
    // affect output — even though the integrator still accumulates internally.
    EXPECT_DOUBLE_EQ(pid.update(0.5, 0.1), 0.5);
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.05); // accumulates: 0.5 * 0.1
}
