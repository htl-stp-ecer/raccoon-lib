//
// Created by tobias on 11/9/25.
//

#pragma once

#include <memory>

#include "odometry/odometry.hpp"
#include "hal/IIMU.hpp"
#include "kinematics/kinematics.hpp"

namespace libstp::odometry::fused
{
    /**
     * Configuration for FusedOdometry
     */
    struct FusedOdometryConfig {
        int imu_ready_timeout_ms;
        bool enable_accel_fusion;
        float bemf_trust;

        FusedOdometryConfig(int imu_ready_timeout_ms = 1000,
                            bool enable_accel_fusion = true,
                            float bemf_trust = 0.95f)
            : imu_ready_timeout_ms(imu_ready_timeout_ms)
            , enable_accel_fusion(enable_accel_fusion)
            , bemf_trust(bemf_trust)
        {}
    };

    /**
     * Fused odometry implementation combining IMU orientation with kinematics velocity
     *
     * This odometry model:
     * - Integrates IMU orientation tracking directly (handles auto-calibration)
     * - Integrates body velocities from kinematics to estimate position
     * - Does NOT directly access wheel encoders - uses IKinematics abstraction
     * - Provides 2D position tracking by integrating velocities in the world frame
     * - Tracks distances from origin in forward/lateral directions
     * - Provides all coordinate frame transformations
     *
     * Position drift will accumulate over time without external corrections.
     * Use reset() with external position references (AprilTags, vision, etc.) to correct drift.
     */
    class FusedOdometry : public IOdometry
    {
    private:
        // Configuration
        FusedOdometryConfig config_;

        // Hardware interfaces (shared ownership)
        std::shared_ptr<hal::imu::IIMU> imu_;
        std::shared_ptr<kinematics::IKinematics> kinematics_;

        // Current state
        Eigen::Vector3f position_;           // Current position in world frame
        Eigen::Quaternionf orientation_;     // Current orientation in world frame

        // Origin tracking (set by reset)
        Eigen::Vector3f origin_position_;    // Position at last reset
        Eigen::Quaternionf origin_orientation_; // Orientation at last reset

        // IMU initialization
        Eigen::Quaternionf initial_imu_orientation_; // Raw IMU orientation at first update
        bool imu_initialized_;

        // Complementary filter state
        bool fusion_initialized_{false};

        // Helper methods
        [[nodiscard]] Eigen::Quaternionf getRelativeOrientation() const;

    public:
        /**
         * Construct fused odometry with IMU and kinematics
         * @param imu Shared pointer to IMU instance
         * @param kinematics Shared pointer to kinematics model (provides velocity estimates)
         * @param config Configuration options (optional)
         */
        FusedOdometry(std::shared_ptr<hal::imu::IIMU> imu,
                      std::shared_ptr<kinematics::IKinematics> kinematics,
                      FusedOdometryConfig config = {});

        // IOdometry interface implementation
        void update(double dt) override;
        [[nodiscard]] foundation::Pose getPose() const override;
        [[nodiscard]] DistanceFromOrigin getDistanceFromOrigin() const override;
        [[nodiscard]] double getHeading() const override;
        [[nodiscard]] double getHeadingError(double target_heading_rad) const override;
        [[nodiscard]] Eigen::Vector3f transformToBodyFrame(const Eigen::Vector3f& world_vec) const override;
        [[nodiscard]] Eigen::Vector3f transformToWorldFrame(const Eigen::Vector3f& body_vec) const override;
        void reset(const foundation::Pose& pose) override;
        void reset() override;
    };
}
