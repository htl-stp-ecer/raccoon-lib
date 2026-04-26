#pragma once
#include <functional>
#include "pch.hpp"

/// Angle conversion helpers shared across robotics code.
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

namespace libstp::math {
    using InterpolationFunction = std::function<float(float, float, float)>;

    /// Linear interpolation between `a` and `b` for parameter `t`.
    float lerp(float a, float b, float t);

    /// Cosine-smoothed interpolation between `a` and `b` for parameter `t`.
    float easeInOut(float a, float b, float t);

    /// Clamp a float to the inclusive range [`min`, `max`].
    float clampf(float value, float min, float max);

    /// Clamp a double to the inclusive range [`min`, `max`].
    double clampDouble(double value, double min, double max);

    /// Clamp an integer to the inclusive range [`min`, `max`].
    int clampInt(int value, int min, int max);

    /// Return -1, 0, or 1 depending on the sign of `value`.
    int sign(int value);

    /// Return -1.0f, 0.0f, or 1.0f depending on the sign of `value`.
    float signf(float value);

    /// Return the smallest absolute angular separation between `a` and `b` in radians.
    float minimalAngleDifference(float a, float b);

    /// Convert a quaternion to roll, pitch, yaw in radians.
    std::tuple<double, double, double> quaternionToEuler(Eigen::Quaterniond q);
}
