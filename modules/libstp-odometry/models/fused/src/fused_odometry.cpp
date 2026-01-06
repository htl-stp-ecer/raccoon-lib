//
// Created by tobias on 11/9/25.
//

#include "odometry/fused/fused_odometry.hpp"
#include "odometry/angle_utils.hpp"
#include "foundation/config.hpp"

namespace libstp::odometry::fused
{
    FusedOdometry::FusedOdometry(std::shared_ptr<hal::imu::IMU> imu,
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
        , imu_initialized_(false)
    {
        if (!imu_) throw std::invalid_argument("imu cannot be null");
        if (!kinematics_) throw std::invalid_argument("kinematics cannot be null");

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

        // ===== ORIENTATION: Read and process IMU =====

        // Initialize IMU baseline on first update
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

        // Get relative orientation (from initial IMU reading)
        const Eigen::Quaternionf relative_orientation = getRelativeOrientation();

        // Current world orientation = origin orientation * relative orientation
        orientation_ = origin_orientation_ * relative_orientation;
        orientation_.normalize();

        // ===== POSITION: Integrate velocity from kinematics =====

        // Get body-frame velocity from kinematics (uses wheel encoders internally)
        const foundation::ChassisVelocity body_velocity = kinematics_->estimateState();

        // Create 3D velocity vector in body frame (2D motion, z=0)
        const Eigen::Vector3f v_body(body_velocity.vx, body_velocity.vy, 0.0f);

        // Transform body velocity to world frame using current orientation
        const Eigen::Vector3f v_world = orientation_ * v_body;

        // Integrate position in world frame
        position_ += v_world * static_cast<float>(dt);

        LIBSTP_LOG_TRACE(
            "FusedOdometry::update dt={} body_vel=({:.3f}, {:.3f}) world_vel=({:.3f}, {:.3f}) pos=({:.3f}, {:.3f}) heading={:.3f}",
            dt,
            body_velocity.vx, body_velocity.vy,
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
        // Compute heading relative to origin orientation
        const Eigen::Quaternionf relative_to_origin = origin_orientation_.inverse() * orientation_;
        return extractYaw(relative_to_origin);
    }

    double FusedOdometry::getHeadingError(double target_heading_rad) const
    {
        const double current_heading = getHeading();
        return angularError(current_heading, target_heading_rad);
    }

    Eigen::Vector3f FusedOdometry::transformToBodyFrame(const Eigen::Vector3f& world_vec) const
    {
        return orientation_.inverse() * world_vec;
    }

    Eigen::Vector3f FusedOdometry::transformToWorldFrame(const Eigen::Vector3f& body_vec) const
    {
        return orientation_ * body_vec;
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

        LIBSTP_LOG_TRACE("FusedOdometry::reset to origin");
    }
}
