#include <gtest/gtest.h>
#include <cmath>
#include <tuple>
#include <Eigen/Geometry>
#include "foundation/math.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::math;
using libstp::test::AlgorithmTestFixture;

namespace
{
    constexpr float kF = 1e-5f;
    constexpr double kD = 1e-9;
}

// ---------------------------------------------------------------------------
// lerp
// ---------------------------------------------------------------------------
class MathLerpTest : public AlgorithmTestFixture {};

TEST_F(MathLerpTest, EndpointsExact)
{
    EXPECT_NEAR(lerp(2.0f, 8.0f, 0.0f), 2.0f, kF);
    EXPECT_NEAR(lerp(2.0f, 8.0f, 1.0f), 8.0f, kF);
}

TEST_F(MathLerpTest, Midpoint)
{
    EXPECT_NEAR(lerp(2.0f, 8.0f, 0.5f), 5.0f, kF);
    EXPECT_NEAR(lerp(-4.0f, 4.0f, 0.25f), -2.0f, kF);
}

TEST_F(MathLerpTest, Extrapolation)
{
    // t outside [0,1] extrapolates linearly (no clamping in lerp itself).
    EXPECT_NEAR(lerp(0.0f, 10.0f, 2.0f), 20.0f, kF);
    EXPECT_NEAR(lerp(0.0f, 10.0f, -1.0f), -10.0f, kF);
}

TEST_F(MathLerpTest, DescendingRange)
{
    // a > b: result must move from a toward b (catches sign of (b-a)).
    EXPECT_NEAR(lerp(10.0f, 2.0f, 0.5f), 6.0f, kF);
    EXPECT_NEAR(lerp(10.0f, 2.0f, 0.25f), 8.0f, kF);
}

// ---------------------------------------------------------------------------
// easeInOut
// ---------------------------------------------------------------------------
class MathEaseTest : public AlgorithmTestFixture {};

TEST_F(MathEaseTest, EndpointsAndMidpoint)
{
    // weight = (1 - cos(t*pi)) * 0.5 -> 0 at t=0, 1 at t=1, 0.5 at t=0.5
    EXPECT_NEAR(easeInOut(0.0f, 10.0f, 0.0f), 0.0f, kF);
    EXPECT_NEAR(easeInOut(0.0f, 10.0f, 1.0f), 10.0f, kF);
    EXPECT_NEAR(easeInOut(0.0f, 10.0f, 0.5f), 5.0f, kF);
}

TEST_F(MathEaseTest, QuarterIsSlowerThanLinear)
{
    // At t=0.25: weight = (1 - cos(pi/4))/2 = (1 - 0.70710678)/2 = 0.14644661
    const float expected = 0.0f + (10.0f - 0.0f) * 0.14644660940672621f;
    EXPECT_NEAR(easeInOut(0.0f, 10.0f, 0.25f), expected, 1e-4f);
    // It must ease in: less than the linear value (2.5) at the same t.
    EXPECT_LT(easeInOut(0.0f, 10.0f, 0.25f), 2.5f);
}

TEST_F(MathEaseTest, SymmetricAroundMidpoint)
{
    // Cosine smoothing is symmetric: weight(0.25) + weight(0.75) == 1.0
    const float lo = easeInOut(0.0f, 10.0f, 0.25f);
    const float hi = easeInOut(0.0f, 10.0f, 0.75f);
    EXPECT_NEAR(lo + hi, 10.0f, 1e-4f);
    EXPECT_GT(hi, 7.5f); // eases out: faster than linear past the midpoint
}

// ---------------------------------------------------------------------------
// clampf / clampDouble / clampInt
// ---------------------------------------------------------------------------
class MathClampTest : public AlgorithmTestFixture {};

TEST_F(MathClampTest, FloatBelowAtAbove)
{
    EXPECT_NEAR(clampf(-5.0f, 0.0f, 10.0f), 0.0f, kF);   // below -> min
    EXPECT_NEAR(clampf(15.0f, 0.0f, 10.0f), 10.0f, kF);  // above -> max
    EXPECT_NEAR(clampf(5.0f, 0.0f, 10.0f), 5.0f, kF);    // inside -> unchanged
}

TEST_F(MathClampTest, FloatBoundariesInclusive)
{
    EXPECT_NEAR(clampf(0.0f, 0.0f, 10.0f), 0.0f, kF);
    EXPECT_NEAR(clampf(10.0f, 0.0f, 10.0f), 10.0f, kF);
}

TEST_F(MathClampTest, DoublePrecision)
{
    EXPECT_DOUBLE_EQ(clampDouble(2.5, -1.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(clampDouble(-2.5, -1.0, 1.0), -1.0);
    EXPECT_DOUBLE_EQ(clampDouble(0.123456789, -1.0, 1.0), 0.123456789);
}

TEST_F(MathClampTest, IntClamp)
{
    EXPECT_EQ(clampInt(7, 0, 5), 5);
    EXPECT_EQ(clampInt(-3, 0, 5), 0);
    EXPECT_EQ(clampInt(3, 0, 5), 3);
    EXPECT_EQ(clampInt(0, 0, 5), 0);
    EXPECT_EQ(clampInt(5, 0, 5), 5);
}

// ---------------------------------------------------------------------------
// sign / signf
// ---------------------------------------------------------------------------
class MathSignTest : public AlgorithmTestFixture {};

TEST_F(MathSignTest, IntSign)
{
    EXPECT_EQ(sign(42), 1);
    EXPECT_EQ(sign(-42), -1);
    EXPECT_EQ(sign(0), 0);
    EXPECT_EQ(sign(1), 1);
    EXPECT_EQ(sign(-1), -1);
}

TEST_F(MathSignTest, FloatSign)
{
    EXPECT_NEAR(signf(3.14f), 1.0f, kF);
    EXPECT_NEAR(signf(-3.14f), -1.0f, kF);
    EXPECT_NEAR(signf(0.0f), 0.0f, kF);
    // Tiny non-zero values still produce full unit magnitude.
    EXPECT_NEAR(signf(1e-30f), 1.0f, kF);
    EXPECT_NEAR(signf(-1e-30f), -1.0f, kF);
}

// ---------------------------------------------------------------------------
// minimalAngleDifference
// ---------------------------------------------------------------------------
class MathAngleDiffTest : public AlgorithmTestFixture {};

TEST_F(MathAngleDiffTest, SameAngleZero)
{
    EXPECT_NEAR(minimalAngleDifference(1.0f, 1.0f), 0.0f, kF);
    EXPECT_NEAR(minimalAngleDifference(0.0f, 0.0f), 0.0f, kF);
}

TEST_F(MathAngleDiffTest, SmallPositiveDifference)
{
    EXPECT_NEAR(minimalAngleDifference(0.0f, 0.5f), 0.5f, kF);
    EXPECT_NEAR(minimalAngleDifference(0.5f, 0.0f), 0.5f, kF); // symmetric
}

TEST_F(MathAngleDiffTest, WrapsToShortestArc)
{
    // 0 and 3pi/2 are 90 deg apart the short way (pi/2), not 270 deg.
    EXPECT_NEAR(minimalAngleDifference(0.0f, static_cast<float>(3 * M_PI / 2)),
                static_cast<float>(M_PI / 2), 1e-4f);
}

TEST_F(MathAngleDiffTest, MaxIsPi)
{
    // Opposite directions -> exactly pi.
    EXPECT_NEAR(minimalAngleDifference(0.0f, static_cast<float>(M_PI)),
                static_cast<float>(M_PI), 1e-4f);
}

TEST_F(MathAngleDiffTest, NormalisesNegativeInputs)
{
    // -pi/2 normalises to 3pi/2; vs pi/2 the difference is pi.
    EXPECT_NEAR(minimalAngleDifference(static_cast<float>(-M_PI / 2),
                                       static_cast<float>(M_PI / 2)),
                static_cast<float>(M_PI), 1e-4f);
}

TEST_F(MathAngleDiffTest, ModuloWrapNearTwoPi)
{
    // a just under 2pi, b just over 0 -> tiny difference across the wrap seam.
    const float a = static_cast<float>(2 * M_PI - 0.01);
    const float b = 0.01f;
    EXPECT_NEAR(minimalAngleDifference(a, b), 0.02f, 1e-3f);
}

TEST_F(MathAngleDiffTest, NegativeInputNormalisesNearZero)
{
    // b = -0.02 normalises to (2pi - 0.02); vs a = +0.02 the short arc is 0.04.
    // Exercises the "angle < 0 -> += 2pi" normalisation branch with the exact
    // 2*M_PI term (not 2/M_PI), and confirms the strict < 0 test.
    const float a = 0.02f;
    const float b = -0.02f;
    EXPECT_NEAR(minimalAngleDifference(a, b), 0.04f, 1e-3f);
}

TEST_F(MathAngleDiffTest, ExactlyZeroNotShifted)
{
    // a normalises to exactly 0 (fmod(2pi,2pi)=0, and 0 < 0 is false so no
    // shift). b = 0. Difference must be 0, catching a `<=` mutation that would
    // wrongly add 2pi to a zero angle.
    const float a = static_cast<float>(2 * M_PI);
    EXPECT_NEAR(minimalAngleDifference(a, 0.0f), 0.0f, 1e-3f);
}

TEST_F(MathAngleDiffTest, EquivalentAfterFullTurn)
{
    // Adding a full revolution must not change the result (fmod wrap).
    const float base = minimalAngleDifference(0.3f, 1.1f);
    const float wrapped = minimalAngleDifference(0.3f + static_cast<float>(2 * M_PI),
                                                 1.1f);
    EXPECT_NEAR(base, wrapped, 1e-3f);
}

// ---------------------------------------------------------------------------
// quaternionToEuler
// ---------------------------------------------------------------------------
class MathQuatTest : public AlgorithmTestFixture {};

TEST_F(MathQuatTest, IdentityIsAllZero)
{
    auto [roll, pitch, yaw] = quaternionToEuler(Eigen::Quaterniond::Identity());
    EXPECT_NEAR(roll, 0.0, 1e-9);
    EXPECT_NEAR(pitch, 0.0, 1e-9);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
}

TEST_F(MathQuatTest, PureYaw)
{
    const double yawAngle = M_PI / 4; // 45 deg about Z
    Eigen::Quaterniond q(Eigen::AngleAxisd(yawAngle, Eigen::Vector3d::UnitZ()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, yawAngle, 1e-6);
}

TEST_F(MathQuatTest, PureRoll)
{
    const double rollAngle = M_PI / 6; // 30 deg about X
    Eigen::Quaterniond q(Eigen::AngleAxisd(rollAngle, Eigen::Vector3d::UnitX()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(roll, rollAngle, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST_F(MathQuatTest, PurePitch)
{
    const double pitchAngle = M_PI / 6; // 30 deg about Y
    Eigen::Quaterniond q(Eigen::AngleAxisd(pitchAngle, Eigen::Vector3d::UnitY()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, pitchAngle, 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST_F(MathQuatTest, GimbalLockPositive90Pitch)
{
    // +90 deg pitch about Y, with a 0.3 rad roll so the +90 gimbal branch
    // (roll = atan2(m01, m02), yaw = 0) is DISTINGUISHABLE from both the -90
    // branch (would give roll - pi) and the general branch (roll != 0.3).
    Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(pitch, M_PI / 2, 1e-5);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
    EXPECT_NEAR(roll, 0.3, 1e-5);   // exact +90 gimbal-branch value
}

TEST_F(MathQuatTest, GimbalLockNegative90Pitch)
{
    // -90 deg pitch about Y, with a 0.3 rad roll. The -90 branch uses
    // roll = atan2(-m01, -m02) and yaw = 0. Picking a non-zero roll makes the
    // result differ from the +90 branch and the general branch.
    Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(pitch, -M_PI / 2, 1e-5);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
    EXPECT_NEAR(roll, 0.3, 1e-5);   // exact -90 gimbal-branch value
}

TEST_F(MathQuatTest, GimbalLockPure90PitchHasZeroRoll)
{
    // Spec (not impl-echo): a pure +90 deg pitch about Y has no roll or yaw,
    // so the +90 gimbal-lock branch must report roll == 0 and yaw == 0.
    // Derived from the geometry, independent of how the branch computes roll.
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(pitch, M_PI / 2, 1e-5);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
    EXPECT_NEAR(roll, 0.0, 1e-5);
}

TEST_F(MathQuatTest, GimbalLockRecoversAppliedRollNegative)
{
    // Spec (not impl-echo): at +90 pitch the function pins yaw to 0 and reports
    // the residual rotation as roll. Applying a known roll of -0.7 rad about X
    // (post-multiplied, the Z*Y*X decode convention) before the +90 pitch must
    // be recovered as roll == -0.7. The negative magnitude (distinct from the
    // sibling +0.3 test) pins the SIGN of the +90 branch arithmetic: the -90
    // branch negates the off-diagonal terms and would return the wrong value,
    // so this distinguishes the two gimbal branches and the general branch.
    const double appliedRoll = -0.7;
    Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(appliedRoll, Eigen::Vector3d::UnitX()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(pitch, M_PI / 2, 1e-5);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
    EXPECT_NEAR(roll, appliedRoll, 1e-5);
}

TEST_F(MathQuatTest, CombinedRollPitchYawRoundTrip)
{
    // Compose a rotation from known r/p/y (away from gimbal lock) using the
    // same Z*Y*X convention the function decodes, then verify recovery.
    const double r = 0.2, p = 0.3, y = -0.5;
    Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()));
    auto [roll, pitch, yaw] = quaternionToEuler(q);
    EXPECT_NEAR(roll, r, 1e-6);
    EXPECT_NEAR(pitch, p, 1e-6);
    EXPECT_NEAR(yaw, y, 1e-6);
    (void)kD;
}
