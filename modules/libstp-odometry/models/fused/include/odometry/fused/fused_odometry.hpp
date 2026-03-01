//
// Created by tobias on 11/9/25.
//

#pragma once

#include <memory>
#include <string>

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

        /// Which axis to use for yaw rate extraction.
        /// "world_z" (default) uses gyro Z (already in world frame).
        /// "body_x", "body_y", "body_z" use the raw body-frame gyro component directly.
        std::string turn_axis;

        FusedOdometryConfig(const int imu_ready_timeout_ms = 1000,
                            const bool enable_accel_fusion = true,
                            const float bemf_trust = 0.95f,
                            std::string turn_axis = "world_z")
            : imu_ready_timeout_ms(imu_ready_timeout_ms)
            , enable_accel_fusion(enable_accel_fusion)
            , bemf_trust(bemf_trust)
            , turn_axis(std::move(turn_axis))
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

        // Origin tracking (set by reset)
        Eigen::Vector3f origin_position_;    // Position at last reset
        double origin_heading_{0.0};         // Heading at last reset (radians)

        // IMU heading tracking
        double initial_imu_heading_{0.0};    // Firmware heading at first update (radians)
        double last_imu_heading_{0.0};       // Latest firmware heading (radians)
        bool imu_initialized_{false};

        // Complementary filter state
        bool fusion_initialized_{false};

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
        void reset(const foundation::Pose& pose) override;
        void reset() override;
    };
}
