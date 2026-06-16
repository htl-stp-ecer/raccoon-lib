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

    /**
     * @brief Which sensor source currently backs the primary pose estimate.
     *
     * The wombat platform automatically switches to the external calibration
     * board when it is detected, because its optical-flow + IMU fusion is far
     * more accurate than the cheap on-board dead reckoning. The internal
     * estimate stays available through the `getInternal*()` accessors so it can
     * be tuned against the external reference.
     */
    enum class OdometrySource
    {
        /// Cheap on-board dead reckoning (STM32 wheel odometry / sim).
        Internal,
        /// External calibration board (optical-flow + IMU fusion).
        CalibrationBoard,
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

        /**
         * Select which source should back getPose()/getHeading()/etc. when
         * available.
         *
         * Implementations may fall back to ::Internal when the requested
         * source is unavailable (for example when the calibration board is not
         * connected). Callers can inspect getActiveSource() to see which source
         * is actually in use right now.
         *
         * @param source Preferred odometry source.
         */
        virtual void setPreferredSource(OdometrySource source)
        {
            (void)source;
        }

        /**
         * Get the caller-selected preferred source.
         *
         * Implementations that do not support source switching always report
         * ::Internal here.
         *
         * @return Preferred odometry source.
         */
        [[nodiscard]] virtual OdometrySource getPreferredSource() const
        {
            return OdometrySource::Internal;
        }

        /**
         * Which source currently backs getPose()/getHeading()/etc.
         *
         * Implementations that have no external reference (mock, sim) always
         * report ::Internal. Implementations that support a secondary source
         * may still report ::Internal here when that source is not preferred
         * or not currently available.
         *
         * @return Active odometry source.
         */
        [[nodiscard]] virtual OdometrySource getActiveSource() const
        {
            return OdometrySource::Internal;
        }

        /**
         * Get the internal (on-board dead-reckoning) pose, regardless of which
         * source is currently active.
         *
         * When no external source exists this is identical to getPose(). On the
         * wombat platform this always returns the cheap STM32 estimate even when
         * the accurate calibration board is driving getPose() — letting callers
         * read both side by side to tune the internal model against the
         * external reference.
         *
         * @return Internal planar pose in world coordinates (meters / radians).
         */
        [[nodiscard]] virtual foundation::Pose getInternalPose() const
        {
            return getPose();
        }

        /**
         * Get the internal (on-board dead-reckoning) heading in radians,
         * regardless of which source is currently active.
         * @return Internal heading in radians, range [-π, π].
         */
        [[nodiscard]] virtual double getInternalHeading() const
        {
            return getHeading();
        }

        /**
         * Get distance-from-origin computed from the internal pose, regardless
         * of which source is currently active.
         * @return Forward, lateral, and straight-line distances from origin (m).
         */
        [[nodiscard]] virtual DistanceFromOrigin getInternalDistanceFromOrigin() const
        {
            return getDistanceFromOrigin();
        }
    };
}
