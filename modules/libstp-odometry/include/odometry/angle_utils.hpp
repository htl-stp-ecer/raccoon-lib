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
 * @brief Compute angular error using quaternion math (robust, no gimbal lock)
 *
 * Computes the yaw component of the rotation needed to go from current orientation
 * to target orientation. This method uses quaternion algebra and is immune to
 * gimbal lock issues.
 *
 * @param current Current orientation quaternion (should be normalized)
 * @param target Target orientation quaternion (should be normalized)
 * @return Signed yaw error in radians, range [-π, π]
 *         - Positive: turn CCW
 *         - Negative: turn CW
 *
 * This is the RECOMMENDED method for heading error computation in motion control
 * as it:
 * - Handles quaternion hemisphere ambiguity (q and -q represent same rotation)
 * - Avoids gimbal lock
 * - Works correctly for all orientations
 *
 * Use this function as input to heading PID controllers when working with
 * quaternion-based odometry.
 */
inline double quaternionAngularError(const Eigen::Quaternionf& current,
                                      const Eigen::Quaternionf& target) {
    // Normalize to ensure valid quaternions
    Eigen::Quaternionf current_norm = current.normalized();
    Eigen::Quaternionf target_norm = target.normalized();

    // Handle quaternion hemisphere: q and -q represent the same rotation
    // Choose the hemisphere that minimizes the rotation angle
    if (target_norm.dot(current_norm) < 0.0f) {
        target_norm.coeffs() *= -1.0f;
    }

    // Compute relative rotation: current -> target
    // q_error represents the rotation needed to go from current to target
    Eigen::Quaternionf q_error = target_norm * current_norm.conjugate();

    // Extract yaw angle from error quaternion
    // For a quaternion q = [w, x, y, z], the yaw (rotation around z-axis) is:
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    //
    // However, for small angles around z-axis, we can use the simplified form:
    // yaw ≈ 2 * atan2(z, w)
    //
    // This is valid when pitch and roll are small, which is typical for
    // ground robots. For full 3D orientation, use the complete formula.
    const double yaw_error = 2.0 * std::atan2(static_cast<double>(q_error.z()),
                                               static_cast<double>(q_error.w()));

    // Wrap to [-π, π] to ensure consistent output
    return wrapAngle(yaw_error);
}

/**
 * @brief Extract yaw angle from a quaternion
 *
 * Extracts the yaw (rotation around z-axis) component from a quaternion.
 *
 * @param orientation Orientation quaternion (should be normalized)
 * @return Yaw angle in radians, range [-π, π]
 *
 * Note: This assumes a right-handed coordinate system with z-axis pointing up.
 * For ground robots with small pitch/roll, this gives accurate heading.
 */
inline double extractYaw(const Eigen::Quaternionf& orientation) {
    const Eigen::Quaternionf q = orientation.normalized();

    // Yaw extraction using atan2 for proper quadrant handling
    const double siny_cosp = 2.0 * (static_cast<double>(q.w() * q.z()) +
                                     static_cast<double>(q.x() * q.y()));
    const double cosy_cosp = 1.0 - 2.0 * (static_cast<double>(q.y() * q.y()) +
                                           static_cast<double>(q.z() * q.z()));

    // Negate to match robotics convention: positive yaw = CCW when viewed from above
    // The IMU/coprocessor uses aviation convention (NED) where positive yaw = CW
    return -std::atan2(siny_cosp, cosy_cosp);
}

} // namespace odometry
} // namespace libstp
