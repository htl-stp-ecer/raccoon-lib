#include <gtest/gtest.h>

#include "libstp/sim/Pose2D.hpp"

#include <cmath>

using namespace libstp::sim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTol = 1e-4f;

    ::testing::AssertionResult PoseNear(const Pose2D& a, const Pose2D& b, float tol = kTol)
    {
        const float dTheta = std::abs(normalizeAngle(a.theta - b.theta));
        if (std::abs(a.x - b.x) < tol && std::abs(a.y - b.y) < tol && dTheta < tol)
        {
            return ::testing::AssertionSuccess();
        }
        return ::testing::AssertionFailure()
            << "got (" << a.x << ", " << a.y << ", " << a.theta << ") "
            << "expected (" << b.x << ", " << b.y << ", " << b.theta << ")";
    }
}

TEST(Pose2DTest, NormalizeAngleWrapsToPiRange)
{
    EXPECT_NEAR(normalizeAngle(0.0f), 0.0f, kTol);
    EXPECT_NEAR(normalizeAngle(kPi), kPi, kTol);
    EXPECT_NEAR(normalizeAngle(-kPi + 0.001f), -kPi + 0.001f, kTol);
    EXPECT_NEAR(normalizeAngle(3.0f * kPi), kPi, kTol);
    EXPECT_NEAR(normalizeAngle(-3.0f * kPi), -kPi, kTol);
}

TEST(Pose2DTest, ForwardMoveZeroHeading)
{
    const Pose2D start{0.0f, 0.0f, 0.0f};
    EXPECT_TRUE(PoseNear(forwardMove(start, 30.0f), {30.0f, 0.0f, 0.0f}));
}

TEST(Pose2DTest, ForwardMoveNinetyDegrees)
{
    const Pose2D start{0.0f, 0.0f, kPi / 2.0f};
    EXPECT_TRUE(PoseNear(forwardMove(start, 30.0f), {0.0f, 30.0f, kPi / 2.0f}));
}

TEST(Pose2DTest, StrafeMoveIsPerpendicular)
{
    const Pose2D start{0.0f, 0.0f, 0.0f};
    // +strafe = left = +Y when heading is +X
    EXPECT_TRUE(PoseNear(strafeMove(start, 10.0f), {0.0f, 10.0f, 0.0f}));
}

TEST(Pose2DTest, ApplyLocalDeltaCombined)
{
    // Start heading +X. Move (5, 3) locally = (5, 3) world. Then rotate 90°.
    const Pose2D start{0.0f, 0.0f, 0.0f};
    const Pose2D out = applyLocalDelta(start, 5.0f, 3.0f, kPi / 2.0f);
    EXPECT_TRUE(PoseNear(out, {5.0f, 3.0f, kPi / 2.0f}));
}

TEST(Pose2DTest, ApplyLocalDeltaRotatedFrame)
{
    // Already facing +Y. Local forward = world +Y.
    const Pose2D start{10.0f, 10.0f, kPi / 2.0f};
    const Pose2D out = applyLocalDelta(start, 5.0f, 0.0f, 0.0f);
    EXPECT_TRUE(PoseNear(out, {10.0f, 15.0f, kPi / 2.0f}));
}

TEST(Pose2DTest, RotateOnlyChangesTheta)
{
    const Pose2D start{10.0f, 20.0f, 0.0f};
    const Pose2D out = rotate(start, kPi / 4.0f);
    EXPECT_NEAR(out.x, 10.0f, kTol);
    EXPECT_NEAR(out.y, 20.0f, kTol);
    EXPECT_NEAR(out.theta, kPi / 4.0f, kTol);
}

TEST(Pose2DTest, ApplyLocalDeltaRoundTrip)
{
    // Apply a delta then apply the inverse delta in the new local frame,
    // should return to the original pose.
    const Pose2D start{1.0f, 2.0f, 0.3f};
    const Pose2D mid = applyLocalDelta(start, 5.0f, 2.0f, 0.1f);
    const Pose2D back = applyLocalDelta(mid, -5.0f, -2.0f, -0.1f);
    // Inverse delta is in the *new* frame, so this won't be an exact round-trip
    // except for the theta component.
    EXPECT_NEAR(normalizeAngle(back.theta - start.theta), 0.0f, kTol);
}
