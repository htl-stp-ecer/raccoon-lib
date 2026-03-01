#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace libstp {
namespace odometry {

/**
 * @brief Utility functions for angle computations with proper wraparound handling
 *
 * These functions are designed to work with PID controllers and motion planning
 * where angular errors must be computed in the shortest direction.
 */

/**
 * @brief Wrap an angle to the range [-π, π]
 * @param angle_rad Angle in radians
 * @return Wrapped angle in radians, range [-π, π]
 */
inline double wrapAngle(double angle_rad) {
    // Use fmod to bring into approximate range
    double wrapped = std::fmod(angle_rad + M_PI, 2.0 * M_PI);

    // Handle negative results from fmod
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }

    // Shift back to [-π, π]
    return wrapped - M_PI;
}

/**
 * @brief Compute shortest angular error from current to target heading
 *
 * Returns the signed angular difference that represents the shortest path
 * from current to target. Properly handles all angle wraparounds.
 *
 * @param current_rad Current angle in radians
 * @param target_rad Target angle in radians
 * @return Signed error in radians, range [-π, π]
 *         - Positive: turn counter-clockwise (CCW) to reach target
 *         - Negative: turn clockwise (CW) to reach target
 *
 * Example:
 *   current = 170°, target = -170° → error = 20° (turn CCW 20°, not CW 340°)
 *   current = -170°, target = 170° → error = -20° (turn CW 20°, not CCW 340°)
 *
 * Use this function as input to heading PID controllers.
 */
inline double angularError(double current_rad, double target_rad) {
    return wrapAngle(target_rad - current_rad);
}

/**
 * @brief Extract heading (horizontal-plane direction) from a quaternion
 *
 * Projects the body X-axis (forward) through the quaternion into world frame,
 * then measures the angle of this vector on the world XY plane.
 *
 * This method is IMMUNE to gimbal lock because it uses vector projection
 * rather than Euler angle decomposition. It works correctly regardless of
 * body tilt (flat robot, robot on its side, etc.).
 *
 * @param orientation Body-to-world orientation quaternion (should be normalized)
 * @return Heading angle in radians, range [-π, π]
 *         0 = forward along +X, π/2 = forward along +Y (CCW from above)
 *
 * Note: Degenerates only when body X is exactly vertical (pointing straight
 * up/down), which never happens for a ground robot.
 */
inline double extractHeading(const Eigen::Quaternionf& orientation) {
    // Rotate body X (forward) into world frame
    const Eigen::Vector3f forward = orientation.normalized() * Eigen::Vector3f::UnitX();

    // Measure direction on world XY plane
    return std::atan2(static_cast<double>(forward.y()),
                      static_cast<double>(forward.x()));
}

} // namespace odometry
} // namespace libstp
