//
// Created by tobias on 4/21/25.
//

#pragma once

#include "pch.hpp"

namespace libstp::foundation {

    using Vector3f = Eigen::Vector3f;
    using Quaternionf = Eigen::Quaternionf;

    struct Pose {
        Vector3f position;
        Quaternionf orientation;
    };

    struct ChassisCmd { double vx, vy, wz; };
    struct ChassisState { double vx, vy, wz; };
    struct ChassisVel {
        double vx;
        double vy;
        double w;
    };


} // namespace foundation
