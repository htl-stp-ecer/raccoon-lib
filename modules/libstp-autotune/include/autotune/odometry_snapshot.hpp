#pragma once

#include "hal/odometry.hpp"

#include <cmath>
#include <numbers>

namespace libstp::autotune
{
    /// Snapshot a pose at the start of a trial and project subsequent absolute
    /// odometry reads into that snapshot's body frame. Replaces the historical
    /// `odometry.reset()` call at trial start — autotune steps are responsible
    /// for being "relative" themselves; odometry stays absolute.
    struct OdometrySnapshot
    {
        double x{0.0};
        double y{0.0};
        double heading{0.0};

        static OdometrySnapshot capture(const odometry::IOdometry& odom)
        {
            const auto pose = odom.getPose();
            return {static_cast<double>(pose.position.x()),
                    static_cast<double>(pose.position.y()),
                    static_cast<double>(pose.heading)};
        }

        /// Return (forward, lateral) signed distances in the snapshot body frame.
        /// Matches `IOdometry::getDistanceFromOrigin` convention.
        [[nodiscard]] std::pair<double, double>
        projectBodyFrame(const odometry::IOdometry& odom) const
        {
            const auto pose = odom.getPose();
            const double dx = static_cast<double>(pose.position.x()) - x;
            const double dy = static_cast<double>(pose.position.y()) - y;
            const double cos_h = std::cos(heading);
            const double sin_h = std::sin(heading);
            const double forward = dx * cos_h + dy * sin_h;
            const double lateral = -dx * sin_h + dy * cos_h;
            return {forward, lateral};
        }

        /// Return signed heading delta wrapped to [-pi, pi].
        [[nodiscard]] double headingDelta(const odometry::IOdometry& odom) const
        {
            const double current = odom.getHeading();
            return std::remainder(current - heading, 2.0 * std::numbers::pi);
        }
    };
}
