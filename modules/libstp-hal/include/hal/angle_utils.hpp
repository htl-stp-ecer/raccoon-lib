#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace libstp {
namespace odometry {

inline double wrapAngle(double angle_rad) {
    double wrapped = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped - M_PI;
}

inline double angularError(double current_rad, double target_rad) {
    return wrapAngle(target_rad - current_rad);
}

inline double extractHeading(const Eigen::Quaternionf& orientation) {
    const Eigen::Vector3f forward = orientation.normalized() * Eigen::Vector3f::UnitX();
    return std::atan2(static_cast<double>(forward.y()),
                      static_cast<double>(forward.x()));
}

} // namespace odometry
} // namespace libstp
