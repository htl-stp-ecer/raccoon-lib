#pragma once

#include <cmath>

namespace libstp::sim
{
    struct Vec2
    {
        float x{0.0f};
        float y{0.0f};
    };

    /// 2D robot pose. x,y in cm; theta in radians (0 = +X, positive = CCW).
    /// Matches the web-ide Pose2D definition so ftmap scenes and planner output
    /// can be fed through the same math without translation.
    struct Pose2D
    {
        float x{0.0f};
        float y{0.0f};
        float theta{0.0f};
    };

    inline float normalizeAngle(float angle) noexcept
    {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 2.0f * kPi;
        while (angle > kPi) angle -= kTwoPi;
        while (angle < -kPi) angle += kTwoPi;
        return angle;
    }

    /// Apply a delta expressed in the robot's local frame (forward, left, yaw).
    inline Pose2D applyLocalDelta(const Pose2D& pose, float dxLocal, float dyLocal, float dTheta) noexcept
    {
        const float c = std::cos(pose.theta);
        const float s = std::sin(pose.theta);
        return {
            pose.x + dxLocal * c - dyLocal * s,
            pose.y + dxLocal * s + dyLocal * c,
            normalizeAngle(pose.theta + dTheta),
        };
    }

    inline Pose2D forwardMove(const Pose2D& pose, float distanceCm) noexcept
    {
        return {
            pose.x + distanceCm * std::cos(pose.theta),
            pose.y + distanceCm * std::sin(pose.theta),
            pose.theta,
        };
    }

    inline Pose2D strafeMove(const Pose2D& pose, float distanceCm) noexcept
    {
        constexpr float kHalfPi = 1.57079632679489661923f;
        const float perp = pose.theta + kHalfPi;
        return {
            pose.x + distanceCm * std::cos(perp),
            pose.y + distanceCm * std::sin(perp),
            pose.theta,
        };
    }

    inline Pose2D rotate(const Pose2D& pose, float angleRad) noexcept
    {
        return {pose.x, pose.y, normalizeAngle(pose.theta + angleRad)};
    }
}
