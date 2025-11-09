//
// Created by tobias on 11/9/25.
//

#include "odometry/fused/fused_odometry.hpp"
#include "foundation/config.hpp"

namespace libstp::odometry::fused
{
    FusedOdometry::FusedOdometry(hal::imu::IMU* imu, kinematics::IKinematics* kinematics)
        : imu_odometry_(imu)
        , kinematics_(kinematics)
        , position_(Eigen::Vector3f::Zero())
    {
        if (!kinematics) throw std::invalid_argument("kinematics cannot be null");
        SPDLOG_INFO("FusedOdometry::ctor initialized with IMU and kinematics");
    }

    FusedOdometry::FusedOdometry(hal::imu::IMU* imu, kinematics::IKinematics* kinematics, const ImuOdometryConfig& config)
        : imu_odometry_(imu, config)
        , kinematics_(kinematics)
        , position_(Eigen::Vector3f::Zero())
    {
        if (!kinematics) throw std::invalid_argument("kinematics cannot be null");
        SPDLOG_INFO("FusedOdometry::ctor initialized with IMU, kinematics, and config");
    }

    void FusedOdometry::update(double dt)
    {
        if (!kinematics_) return;

        // ===== ORIENTATION: Delegate to ImuOdometry =====
        // This handles IMU reading, inversions, initialization, and orientation tracking
        imu_odometry_.update(dt);

        // Get the orientation-only pose from IMU odometry
        const foundation::Pose imu_pose = imu_odometry_.getPose();

        // ===== POSITION: Integrate velocity from kinematics =====

        // Get body-frame velocity from kinematics (uses wheel encoders internally)
        const foundation::ChassisState body_velocity = kinematics_->estimateState();

        // Create 3D velocity vector in body frame (2D motion, z=0)
        const Eigen::Vector3f v_body(body_velocity.vx, body_velocity.vy, 0.0f);

        // Transform body velocity to world frame using current orientation from IMU
        const Eigen::Vector3f v_world = imu_pose.orientation * v_body;

        // Integrate position in world frame
        position_ += v_world * static_cast<float>(dt);

        SPDLOG_TRACE(
            "FusedOdometry::update dt={} body_vel=({}, {}, {}) world_vel=({}, {}, {}) pos=({}, {}, {})",
            dt,
            body_velocity.vx, body_velocity.vy, body_velocity.wz,
            v_world.x(), v_world.y(), v_world.z(),
            position_.x(), position_.y(), position_.z()
        );
    }

    foundation::Pose FusedOdometry::getPose() const
    {
        // Combine position from velocity integration with orientation from IMU
        foundation::Pose pose;
        pose.position = position_;
        pose.orientation = imu_odometry_.getPose().orientation;
        return pose;
    }

    void FusedOdometry::reset(const foundation::Pose& pose)
    {
        position_ = pose.position;
        imu_odometry_.reset(pose);  // Delegate orientation reset to ImuOdometry
        SPDLOG_INFO(
            "FusedOdometry::reset to pose pos=({}, {}, {}) quat=({}, {}, {}, {})",
            pose.position.x(), pose.position.y(), pose.position.z(),
            pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z()
        );
    }

    void FusedOdometry::reset()
    {
        position_ = Eigen::Vector3f::Zero();
        imu_odometry_.reset();  // Delegate orientation reset to ImuOdometry
        SPDLOG_INFO("FusedOdometry::reset to origin");
    }
}
