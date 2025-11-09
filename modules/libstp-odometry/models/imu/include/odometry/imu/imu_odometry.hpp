//
// Created by tobias on 10/23/25.
//

#pragma once

#include "odometry/odometry.hpp"
#include "hal/IMU.hpp"

namespace libstp::odometry::imu
{
    /**
     * Configuration for IMU odometry
     * Allows inverting quaternion components to handle different IMU coordinate frames
     */
    struct ImuOdometryConfig
    {
        bool invert_x = false;  ///< Invert quaternion x component
        bool invert_y = false;  ///< Invert quaternion y component
        bool invert_z = false;  ///< Invert quaternion z component
        bool invert_w = false;  ///< Invert quaternion w component (for 180° rotation)
    };

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
        ImuOdometryConfig config_;

        /**
         * Apply axis inversions to quaternion based on config
         * @param q Input quaternion
         * @return Inverted quaternion (normalized)
         */
        [[nodiscard]] Eigen::Quaternionf applyInversions(const Eigen::Quaternionf& q) const;

    public:
        /**
         * Construct IMU odometry with an IMU sensor
         * @param imu Pointer to IMU instance
         */
        explicit ImuOdometry(hal::imu::IMU* imu);

        /**
         * Construct IMU odometry with an IMU sensor and configuration
         * @param imu Pointer to IMU instance
         * @param config Configuration for axis inversions
         */
        ImuOdometry(hal::imu::IMU* imu, const ImuOdometryConfig& config);

        ~ImuOdometry() override = default;

        void update(double dt) override;
        [[nodiscard]] foundation::Pose getPose() const override;
        void reset(const foundation::Pose& pose) override;
        void reset() override;
    };
}
