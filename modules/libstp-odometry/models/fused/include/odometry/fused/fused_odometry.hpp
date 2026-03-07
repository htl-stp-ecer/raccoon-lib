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
     * Configuration for `FusedOdometry`.
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
     * Use zero-argument reset() to establish a fresh local origin before a motion sequence.
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
         * @param imu Shared IMU instance used for heading and optional integrated velocity
         * @param kinematics Shared kinematics model used for chassis velocity estimation
         * @param config Configuration options controlling IMU readiness and fusion behavior
         */
        FusedOdometry(std::shared_ptr<hal::imu::IIMU> imu,
                      std::shared_ptr<kinematics::IKinematics> kinematics,
                      FusedOdometryConfig config = {});

        // IOdometry interface implementation
        /** Update heading and position estimates using the latest IMU and kinematics data. */
        void update(double dt) override;

        /** Return the current world-frame pose estimate. */
        [[nodiscard]] foundation::Pose getPose() const override;

        /** Return forward, lateral, and straight-line displacement from the last reset origin. */
        [[nodiscard]] DistanceFromOrigin getDistanceFromOrigin() const override;

        /** Return heading relative to the heading stored at the last reset. */
        [[nodiscard]] double getHeading() const override;

        /** Return the raw IMU heading unaffected by reset(). */
        [[nodiscard]] double getAbsoluteHeading() const override;

        /** Return the shortest signed angular error from the current heading to the target. */
        [[nodiscard]] double getHeadingError(double target_heading_rad) const override;

        /** Reset the origin to zero pose and clear encoder and IMU-integrated velocity state. */
        void reset() override;
    };
}
