//
// Created by tobias on 10/23/25.
//

#pragma once

#include "odometry/odometry.hpp"
#include "hal/IMU.hpp"

namespace libstp::odometry::imu
{
    /**
     * IMU-based odometry implementation
     * Estimates pose using IMU orientation data
     * Note: Position estimation is integrated from orientation assuming planar movement
     */
    class ImuOdometry : public IOdometry
    {
    private:
        hal::imu::IMU* imu_;
        foundation::Pose current_pose_;
        Eigen::Quaternionf initial_orientation_;
        bool initialized_;

    public:
        /**
         * Construct IMU odometry with an IMU sensor
         * @param imu Pointer to IMU instance
         */
        explicit ImuOdometry(hal::imu::IMU* imu);
        ~ImuOdometry() override = default;

        void update(double dt) override;
        [[nodiscard]] foundation::Pose getPose() const override;
        void reset(const foundation::Pose& pose) override;
        void reset() override;
    };
}
