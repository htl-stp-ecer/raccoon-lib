#include <gtest/gtest.h>
#include <numbers>
#include "hal/angle_utils.hpp"

using libstp::odometry::angularError;
using libstp::odometry::extractHeading;
using libstp::odometry::wrapAngle;

static constexpr double kPi = std::numbers::pi;

// ---------------------------------------------------------------------------
// wrapAngle — output must be in (-π, π]
// ---------------------------------------------------------------------------

TEST(WrapAngleTest, ZeroStaysZero) {
    EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
}

TEST(WrapAngleTest, PositivePiWrapsToNegativePi) {
    // π maps to -π (the closed end of the interval is -π, open end is +π)
    EXPECT_NEAR(wrapAngle(kPi), -kPi, 1e-12);
}

TEST(WrapAngleTest, SlightlyAbovePiWrapsNegative) {
    EXPECT_NEAR(wrapAngle(kPi + 0.01), -kPi + 0.01, 1e-12);
}

TEST(WrapAngleTest, SlightlyBelowPiStaysPositive) {
    EXPECT_NEAR(wrapAngle(kPi - 0.01), kPi - 0.01, 1e-12);
}

TEST(WrapAngleTest, TwoPiWrapsToZero) {
    EXPECT_NEAR(wrapAngle(2.0 * kPi), 0.0, 1e-12);
}

TEST(WrapAngleTest, NegativeTwoPiWrapsToZero) {
    EXPECT_NEAR(wrapAngle(-2.0 * kPi), 0.0, 1e-12);
}

TEST(WrapAngleTest, NegativeAnglePreserved) {
    EXPECT_NEAR(wrapAngle(-kPi / 2.0), -kPi / 2.0, 1e-12);
}

TEST(WrapAngleTest, LargePositiveAngle) {
    EXPECT_NEAR(wrapAngle(5.0 * kPi), -kPi, 1e-9);
}

TEST(WrapAngleTest, LargeNegativeAngle) {
    EXPECT_NEAR(wrapAngle(-3.0 * kPi), -kPi, 1e-9);
}

// ---------------------------------------------------------------------------
// angularError — shortest-path signed difference
// ---------------------------------------------------------------------------

TEST(AngularErrorTest, ZeroError) {
    EXPECT_DOUBLE_EQ(angularError(1.0, 1.0), 0.0);
}

TEST(AngularErrorTest, PositiveTurn) {
    EXPECT_NEAR(angularError(0.0, kPi / 2.0), kPi / 2.0, 1e-12);
}

TEST(AngularErrorTest, NegativeTurn) {
    EXPECT_NEAR(angularError(0.0, -kPi / 2.0), -kPi / 2.0, 1e-12);
}

TEST(AngularErrorTest, CrossesPositivePiBoundary) {
    // From +170° to -170°: shortest path is CCW through ±180° = +20°, not -340°
    const double current = 170.0 * kPi / 180.0;
    const double target  = -170.0 * kPi / 180.0;
    EXPECT_NEAR(angularError(current, target), 20.0 * kPi / 180.0, 1e-10);
}

TEST(AngularErrorTest, CrossesNegativePiBoundary) {
    // From -170° to +170°: shortest path is CW through ±180° = -20°
    const double current = -170.0 * kPi / 180.0;
    const double target  =  170.0 * kPi / 180.0;
    EXPECT_NEAR(angularError(current, target), -20.0 * kPi / 180.0, 1e-10);
}

TEST(AngularErrorTest, FullCircleIsZero) {
    EXPECT_NEAR(angularError(0.5, 0.5 + 2.0 * kPi), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// extractHeading — Quaternion → yaw in [-π, π]
// ---------------------------------------------------------------------------

TEST(ExtractHeadingTest, IdentityQuaternionIsZero) {
    const Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    EXPECT_NEAR(extractHeading(q), 0.0, 1e-6);
}

TEST(ExtractHeadingTest, NinetyDegreesCCW) {
    const Eigen::Quaternionf q(Eigen::AngleAxisf(
        static_cast<float>(kPi / 2.0), Eigen::Vector3f::UnitZ()));
    EXPECT_NEAR(extractHeading(q), kPi / 2.0, 1e-5);
}

TEST(ExtractHeadingTest, NinetyDegreesCW) {
    const Eigen::Quaternionf q(Eigen::AngleAxisf(
        static_cast<float>(-kPi / 2.0), Eigen::Vector3f::UnitZ()));
    EXPECT_NEAR(extractHeading(q), -kPi / 2.0, 1e-5);
}

TEST(ExtractHeadingTest, OneEightyDegrees) {
    const Eigen::Quaternionf q(Eigen::AngleAxisf(
        static_cast<float>(kPi), Eigen::Vector3f::UnitZ()));
    EXPECT_NEAR(std::abs(extractHeading(q)), kPi, 1e-5);
}

TEST(ExtractHeadingTest, UnnormalizedQuaternion) {
    Eigen::Quaternionf q(2.0f, 0.0f, 0.0f, 0.0f); // unnormalized identity
    EXPECT_NEAR(extractHeading(q), 0.0, 1e-5);
}
