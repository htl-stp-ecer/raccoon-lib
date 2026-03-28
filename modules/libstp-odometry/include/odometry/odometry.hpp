//
// Created by tobias on 10/23/25.
//

#pragma once

#include "foundation/types.hpp"
#include <Eigen/Core>

namespace libstp::odometry
{
    /**
     * @brief Distance measurements from the origin (set by last reset)
     *
     * All distances are in meters, measured from the origin position
     * set by the most recent reset() call.
     */
    struct DistanceFromOrigin
    {
        /**
         * Distance traveled in the initial forward direction (at reset time).
         * Positive = moved forward, negative = moved backward.
         */
        double forward;

        /**
         * Lateral drift perpendicular to initial forward direction.
         * Positive = drifted right, negative = drifted left.
         */
        double lateral;

        /**
         * Straight-line Euclidean distance from origin to current position.
         * Always positive (unsigned distance).
         */
        double straight_line;
    };

    struct IOdometry
    {
        virtual ~IOdometry() = default;

        /**
         * Update the odometry estimate with the latest sensor data
         * @param dt Time delta since last update in seconds
         */
        virtual void update(double dt) = 0;

        /**
         * Get the current pose estimate
         * @return Current planar pose expressed in world coordinates
         */
        [[nodiscard]] virtual foundation::Pose getPose() const = 0;

        /**
         * Get distance measurements from the origin (set by last reset)
         * @return Forward, lateral, and straight-line distances from origin
         */
        [[nodiscard]] virtual DistanceFromOrigin getDistanceFromOrigin() const = 0;

        /**
         * Get current heading (yaw angle) in radians relative to origin orientation
         * @return Heading in radians, range [-π, π]
         *         0 = facing initial forward direction
         *         positive = rotated CCW from initial
         *         negative = rotated CW from initial
         */
        [[nodiscard]] virtual double getHeading() const = 0;

        /**
         * Get heading error to reach a target heading
         *
         * Computes the shortest angular path from current heading to target.
         * Properly handles angle wraparound (e.g., from 170° to -170°).
         *
         * @param target_heading_rad Target heading in radians
         * @return Signed angular error in radians, range [-π, π]
         *         - Positive: turn CCW to reach target
         *         - Negative: turn CW to reach target
         *
         * Use this as input to heading PID controllers.
         */
        [[nodiscard]] virtual double getHeadingError(double target_heading_rad) const = 0;

        /**
         * Get the absolute IMU heading that is unaffected by reset().
         * @return Heading in radians (CCW-positive). Stable across odometry resets.
         */
        [[nodiscard]] virtual double getAbsoluteHeading() const = 0;

        /**
         * Get the cumulative path length (odometer) in meters.
         *
         * Monotonically increasing — NOT affected by reset().
         * Accumulates the Euclidean distance traveled each update cycle,
         * regardless of direction (forward, strafe, curved).
         *
         * @return Total distance traveled in meters since construction
         */
        [[nodiscard]] virtual double getPathLength() const = 0;

        /**
         * Reset the odometry to origin (zero position, identity orientation)
         */
        virtual void reset() = 0;
    };
}
