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

} // namespace foundation
