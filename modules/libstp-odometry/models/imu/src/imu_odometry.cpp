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
    {
        current_pose_.position = foundation::Vector3f::Zero();
        current_pose_.orientation = Eigen::Quaternionf::Identity();
    }

    void ImuOdometry::update(double dt)
    {
        if (!imu_) return;

        // Get current orientation from IMU
        Eigen::Quaternionf raw_orientation = imu_->getOrientation();

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
