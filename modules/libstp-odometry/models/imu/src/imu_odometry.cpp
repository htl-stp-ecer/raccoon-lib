//
// Created by tobias on 10/23/25.
//

#include "odometry/imu/imu_odometry.hpp"

namespace libstp::odometry::imu
{
    ImuOdometry::ImuOdometry(hal::imu::IMU* imu)
        : imu_(imu)
        , current_pose_{}
        , initial_orientation_(Eigen::Quaternionf::Identity())
        , initialized_(false)
        , config_{}
    {
        current_pose_.position = foundation::Vector3f::Zero();
        current_pose_.orientation = Eigen::Quaternionf::Identity();
    }

    ImuOdometry::ImuOdometry(hal::imu::IMU* imu, const ImuOdometryConfig& config)
        : imu_(imu)
        , current_pose_{}
        , initial_orientation_(Eigen::Quaternionf::Identity())
        , initialized_(false)
        , config_(config)
    {
        current_pose_.position = foundation::Vector3f::Zero();
        current_pose_.orientation = Eigen::Quaternionf::Identity();
    }

    Eigen::Quaternionf ImuOdometry::applyInversions(const Eigen::Quaternionf& q) const
    {
        Eigen::Quaternionf result = q;

        // Apply component inversions based on config
        if (config_.invert_w) result.w() = -result.w();
        if (config_.invert_x) result.x() = -result.x();
        if (config_.invert_y) result.y() = -result.y();
        if (config_.invert_z) result.z() = -result.z();

        // Normalize to maintain unit quaternion
        result.normalize();
        return result;
    }

    void ImuOdometry::update(double dt)
    {
        if (!imu_) return;

        // Get current orientation from IMU
        Eigen::Quaternionf raw_orientation = imu_->getOrientation();

        // Apply axis inversions
        raw_orientation = applyInversions(raw_orientation);

        // Initialize on first update
        if (!initialized_)
        {
            initial_orientation_ = raw_orientation;
            initialized_ = true;
        }

        // Compute relative orientation from initial
        current_pose_.orientation = initial_orientation_.inverse() * raw_orientation;
        current_pose_.orientation.normalize();

        // Note: IMU-only odometry cannot estimate position without velocity integration
        // Position remains as set (typically zero or from reset)
        // For full pose estimation, you would need to integrate velocity/acceleration
        // or combine with wheel odometry
    }

    foundation::Pose ImuOdometry::getPose() const
    {
        return current_pose_;
    }

    void ImuOdometry::reset(const foundation::Pose& pose)
    {
        current_pose_ = pose;
        initialized_ = false;
    }

    void ImuOdometry::reset()
    {
        current_pose_.position = foundation::Vector3f::Zero();
        current_pose_.orientation = Eigen::Quaternionf::Identity();
        initialized_ = false;
    }
}
