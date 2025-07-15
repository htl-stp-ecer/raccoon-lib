//
// Created by tobias on 6/8/25.
//
#pragma once
#include <functional>
#include "pch.hpp"

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

namespace libstp::math {
    using InterpolationFunction = std::function<float(float, float, float)>;

    float lerp(float a, float b, float t);

    float easeInOut(float a, float b, float t);

    float clampf(float value, float min, float max);

    double clampDouble(double value, double min, double max);

    int clampInt(int value, int min, int max);

    int sign(int value);

    float signf(float value);

    float minimalAngleDifference(float a, float b);

    std::tuple<double, double, double> quaternionToEuler(Eigen::Quaterniond q);
}
