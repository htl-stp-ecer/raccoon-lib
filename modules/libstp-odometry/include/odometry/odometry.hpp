//
// Created by tobias on 10/23/25.
//

#pragma once

#include "foundation/types.hpp"

namespace libstp::odometry
{
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
         * @return Current pose (position and orientation)
         */
        [[nodiscard]] virtual foundation::Pose getPose() const = 0;

        /**
         * Reset the odometry to a given pose
         * @param pose Initial pose to reset to
         */
        virtual void reset(const foundation::Pose& pose) = 0;

        /**
         * Reset the odometry to origin (zero position, identity orientation)
         */
        virtual void reset() = 0;
    };
}
