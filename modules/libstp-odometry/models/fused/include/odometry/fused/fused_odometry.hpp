//
// Created by tobias on 11/9/25.
//

#pragma once

#include "odometry/imu/imu_odometry.hpp"
#include "kinematics/kinematics.hpp"

namespace libstp::odometry::fused
{
    // Reuse ImuOdometryConfig for IMU configuration
    using ImuOdometryConfig = imu::ImuOdometryConfig;

    /**
     * Fused odometry implementation combining IMU orientation with kinematics velocity
     *
     * This odometry model:
     * - Delegates orientation tracking to ImuOdometry (handles IMU inversions, initialization)
     * - Integrates body velocities from kinematics to estimate position
     * - Does NOT directly access wheel encoders - uses IKinematics abstraction
     * - Provides 2D position tracking by integrating velocities in the world frame
     *
     * Position drift will accumulate over time without external corrections.
     * Use reset() with external position references (AprilTags, vision, etc.) to correct drift.
     */
    class FusedOdometry : public IOdometry
    {
    private:
        imu::ImuOdometry imu_odometry_;  // Handles all IMU orientation tracking
        kinematics::IKinematics* kinematics_;
        Eigen::Vector3f position_;  // Position estimate (orientation comes from imu_odometry_)

    public:
        /**
         * Construct fused odometry with IMU and kinematics
         * @param imu Pointer to IMU instance
         * @param kinematics Pointer to kinematics model (provides velocity estimates)
         */
        FusedOdometry(hal::imu::IMU* imu, kinematics::IKinematics* kinematics);

        /**
         * Construct fused odometry with IMU, kinematics, and configuration
         * @param imu Pointer to IMU instance
         * @param kinematics Pointer to kinematics model (provides velocity estimates)
         * @param config Configuration for IMU axis inversions
         */
        FusedOdometry(hal::imu::IMU* imu, kinematics::IKinematics* kinematics, const ImuOdometryConfig& config);

        ~FusedOdometry() override = default;

        void update(double dt) override;
        [[nodiscard]] foundation::Pose getPose() const override;
        void reset(const foundation::Pose& pose) override;
        void reset() override;
    };
}
