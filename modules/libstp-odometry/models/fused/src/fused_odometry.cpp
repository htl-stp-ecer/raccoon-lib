//
// Created by tobias on 11/9/25.
//

#include "odometry/fused/fused_odometry.hpp"
#include "odometry/angle_utils.hpp"
#include "foundation/config.hpp"
#include <cmath>
#include <algorithm>

namespace libstp::odometry::fused
{
    FusedOdometry::FusedOdometry(std::shared_ptr<hal::imu::IIMU> imu,
                                   std::shared_ptr<kinematics::IKinematics> kinematics,
                                   FusedOdometryConfig config)
        : config_(config)
        , imu_(std::move(imu))
        , kinematics_(std::move(kinematics))
        , position_(Eigen::Vector3f::Zero())
        , orientation_(Eigen::Quaternionf::Identity())
        , origin_position_(Eigen::Vector3f::Zero())
        , origin_orientation_(Eigen::Quaternionf::Identity())
        , initial_imu_orientation_(Eigen::Quaternionf::Identity())
        , last_raw_imu_orientation_(Eigen::Quaternionf::Identity())
        , imu_initialized_(false)
    {
        if (!imu_) throw std::invalid_argument("imu cannot be null");
        if (!kinematics_) throw std::invalid_argument("kinematics cannot be null");

        // Configure which axis to use for yaw rate extraction.
        // "world_z" rotates body gyro via quaternion (requires converged DMP tilt).
        // "body_x/y/z" uses raw body-frame gyro component directly (no quaternion needed).
        imu_->setYawRateAxisMode(config_.turn_axis);

        // Wait for IMU to receive initial data from coprocessor
        if (!imu_->waitForReady(config_.imu_ready_timeout_ms)) {
            LIBSTP_LOG_WARN("FusedOdometry::ctor - IMU not ready after {}ms timeout, proceeding anyway",
                          config_.imu_ready_timeout_ms);
        }

        LIBSTP_LOG_TRACE("FusedOdometry::ctor initialized with IMU and kinematics");
    }

    Eigen::Quaternionf FusedOdometry::getRelativeOrientation() const
    {
        if (!imu_ || !imu_initialized_) {
            return Eigen::Quaternionf::Identity();
        }

        // Get current raw IMU orientation
        Eigen::Quaternionf raw_orientation = imu_->getOrientation();

        // Compute relative orientation from initial IMU reading
        // This makes all orientation relative to the orientation at first update/reset
        Eigen::Quaternionf relative = initial_imu_orientation_.inverse() * raw_orientation;
        return relative.normalized();
    }

    void FusedOdometry::update(double dt)
    {
        if (!imu_ || !kinematics_) return;
        if (dt <= 0.0) return;

        const float dt_f = static_cast<float>(dt);

        // ===== ORIENTATION =====

        if (!imu_initialized_) {
            initial_imu_orientation_ = imu_->getOrientation();
            imu_initialized_ = true;
            LIBSTP_LOG_TRACE(
                "FusedOdometry: IMU initialized with quat=({}, {}, {}, {})",
                initial_imu_orientation_.w(),
                initial_imu_orientation_.x(),
                initial_imu_orientation_.y(),
                initial_imu_orientation_.z()
            );
        }

        const Eigen::Quaternionf raw_imu = imu_->getOrientation();
        last_raw_imu_orientation_ = raw_imu;
        const Eigen::Quaternionf relative_orientation = getRelativeOrientation();
        orientation_ = origin_orientation_ * relative_orientation;
        orientation_.normalize();

        LIBSTP_LOG_DEBUG(
            "ODOM [IMU] q=({:.4f},{:.4f},{:.4f},{:.4f}) q0=({:.4f},{:.4f},{:.4f},{:.4f}) "
            "hdg={:.2f}deg hdg0={:.2f}deg delta={:.2f}deg final={:.2f}deg",
            raw_imu.w(), raw_imu.x(), raw_imu.y(), raw_imu.z(),
            initial_imu_orientation_.w(), initial_imu_orientation_.x(),
            initial_imu_orientation_.y(), initial_imu_orientation_.z(),
            extractHeading(raw_imu) * 180.0 / M_PI,
            extractHeading(initial_imu_orientation_) * 180.0 / M_PI,
            wrapAngle(extractHeading(raw_imu) - extractHeading(initial_imu_orientation_)) * 180.0 / M_PI,
            getHeading() * 180.0 / M_PI
        );

        // ===== BEMF velocity (body frame, read at control loop rate ~20Hz) =====

        const foundation::ChassisVelocity body_velocity_raw = kinematics_->estimateState();
        const Eigen::Vector3f v_bemf_body(body_velocity_raw.vx, body_velocity_raw.vy, 0.0f);

        LIBSTP_LOG_DEBUG(
            "FusedOdometry::update [BEMF] dt={:.4f} v_bemf=({:.4f}, {:.4f}) wz={:.4f}",
            dt, v_bemf_body.x(), v_bemf_body.y(), body_velocity_raw.wz
        );

        // Rotate BEMF to world frame
        const Eigen::Vector3f v_bemf_world = orientation_ * v_bemf_body;

        // ===== COMPLEMENTARY FILTER with firmware-integrated accel velocity =====
        // Accel velocity from firmware is already in world frame

        Eigen::Vector3f v_world = v_bemf_world;

        if (config_.enable_accel_fusion && fusion_initialized_) {
            float vel[3];
            imu_->getIntegratedVelocity(vel);
            const Eigen::Vector3f v_accel_world(vel[0], vel[1], 0.0f);

            const float alpha = std::clamp(config_.bemf_trust, 0.0f, 1.0f);
            v_world = alpha * v_bemf_world + (1.0f - alpha) * v_accel_world;

            LIBSTP_LOG_DEBUG(
                "FusedOdometry::update [FUSE] alpha={:.3f} v_bemf_world=({:.4f}, {:.4f}) "
                "v_accel_world=({:.4f}, {:.4f}) v_fused=({:.4f}, {:.4f})",
                alpha, v_bemf_world.x(), v_bemf_world.y(),
                v_accel_world.x(), v_accel_world.y(),
                v_world.x(), v_world.y()
            );
        }

        fusion_initialized_ = true;

        // Integrate world-frame velocity into position
        position_ += v_world * dt_f;

        LIBSTP_LOG_DEBUG(
            "FusedOdometry::update [RESULT] v_bemf_body=({:.4f}, {:.4f}) v_world=({:.4f}, {:.4f}) "
            "pos=({:.4f}, {:.4f}) heading={:.4f}",
            v_bemf_body.x(), v_bemf_body.y(),
            v_world.x(), v_world.y(),
            position_.x(), position_.y(),
            getHeading()
        );
    }

    foundation::Pose FusedOdometry::getPose() const
    {
        foundation::Pose pose;
        pose.position = position_;
        pose.orientation = orientation_;
        return pose;
    }

    DistanceFromOrigin FusedOdometry::getDistanceFromOrigin() const
    {
        // Displacement from origin in world frame
        const Eigen::Vector3f displacement_world = position_ - origin_position_;

        // Get the forward direction at origin (origin orientation applied to body-frame forward vector)
        const Eigen::Vector3f forward_at_origin = origin_orientation_ * Eigen::Vector3f::UnitX();

        // Get the right direction at origin (origin orientation applied to body-frame right vector)
        // Note: In body frame, right is positive Y (mecanum positive vy moves right)
        const Eigen::Vector3f right_at_origin = origin_orientation_ * Eigen::Vector3f::UnitY();

        // Project displacement onto forward and right directions
        const double forward = displacement_world.dot(forward_at_origin);
        const double lateral = displacement_world.dot(right_at_origin);

        // Straight-line distance (Euclidean)
        const double straight_line = displacement_world.norm();

        return DistanceFromOrigin{forward, lateral, straight_line};
    }

    double FusedOdometry::getHeading() const
    {
        // Use extractHeading (forward-vector projection) instead of extractYaw
        // (ZYX Euler decomposition). extractYaw has gimbal lock at pitch = ±90°,
        // causing massive heading noise when the body is tilted (e.g. body Y up).
        // extractHeading projects body X onto the world XY plane — no gimbal lock.
        if (!imu_initialized_) {
            return extractHeading(origin_orientation_);
        }

        const double imu_heading_delta = wrapAngle(
            extractHeading(last_raw_imu_orientation_) - extractHeading(initial_imu_orientation_));
        return wrapAngle(extractHeading(origin_orientation_) + imu_heading_delta);
    }

    double FusedOdometry::getHeadingError(double target_heading_rad) const
    {
        const double current_heading = getHeading();
        return angularError(current_heading, target_heading_rad);
    }

    void FusedOdometry::reset(const foundation::Pose& pose)
    {
        // Set current state
        position_ = pose.position;
        orientation_ = pose.orientation.normalized();

        // Set new origin to match current state
        origin_position_ = position_;
        origin_orientation_ = orientation_;

        // Re-initialize IMU to current reading
        imu_initialized_ = false;

        // Reset complementary filter state
        fusion_initialized_ = false;
        imu_->resetIntegratedVelocity();

        LIBSTP_LOG_TRACE(
            "FusedOdometry::reset to pose pos=({:.3f}, {:.3f}, {:.3f}) quat=({:.3f}, {:.3f}, {:.3f}, {:.3f})",
            pose.position.x(), pose.position.y(), pose.position.z(),
            pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z()
        );
    }

    void FusedOdometry::reset()
    {
        // Wait for IMU to receive initial data from coprocessor
        if (imu_ && !imu_->waitForReady(config_.imu_ready_timeout_ms)) {
            LIBSTP_LOG_WARN("FusedOdometry::reset - IMU not ready after {}ms timeout, proceeding anyway",
                          config_.imu_ready_timeout_ms);
        }

        if (imu_) {
            imu_->setYawRateAxisMode(config_.turn_axis);
        }

        // Reset encoder tracking to prevent stale position deltas
        if (kinematics_) {
            kinematics_->resetEncoders();
        }

        // Reset to origin (identity pose)
        position_ = Eigen::Vector3f::Zero();
        orientation_ = Eigen::Quaternionf::Identity();

        // Set origin to identity
        origin_position_ = Eigen::Vector3f::Zero();
        origin_orientation_ = Eigen::Quaternionf::Identity();

        // Re-initialize IMU to current reading
        imu_initialized_ = false;

        // Reset complementary filter state
        fusion_initialized_ = false;
        imu_->resetIntegratedVelocity();

        LIBSTP_LOG_TRACE("FusedOdometry::reset to origin");
    }
}
