//
// Created by tobias on 11/9/25.
//

#include "odometry/fused/fused_odometry.hpp"
#include "odometry/angle_utils.hpp" // wrapAngle, angularError
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
        , origin_position_(Eigen::Vector3f::Zero())
    {
        if (!imu_) throw std::invalid_argument("imu cannot be null");
        if (!kinematics_) throw std::invalid_argument("kinematics cannot be null");

        imu_->setYawRateAxisMode(config_.turn_axis);

        // Wait for IMU to receive initial data from coprocessor
        if (!imu_->waitForReady(config_.imu_ready_timeout_ms)) {
            LIBSTP_LOG_WARN("FusedOdometry::ctor - IMU not ready after {}ms timeout, proceeding anyway",
                          config_.imu_ready_timeout_ms);
        }

        LIBSTP_LOG_TRACE("FusedOdometry::ctor initialized with IMU and kinematics");
    }

    void FusedOdometry::update(double dt)
    {
        if (!imu_ || !kinematics_) return;
        if (dt <= 0.0) return;

        const float dt_f = static_cast<float>(dt);

        // ===== HEADING =====

        if (!imu_initialized_) {
            initial_imu_heading_ = imu_->getHeading();
            last_imu_heading_ = initial_imu_heading_;
            imu_initialized_ = true;
            LIBSTP_LOG_TRACE(
                "FusedOdometry: IMU initialized heading={:.2f}deg",
                initial_imu_heading_ * 180.0 / M_PI
            );
        }

        last_imu_heading_ = imu_->getHeading();

        const double heading = getHeading();

        LIBSTP_LOG_DEBUG(
            "ODOM [IMU] hdg={:.2f}deg hdg0={:.2f}deg delta={:.2f}deg final={:.2f}deg",
            last_imu_heading_ * 180.0 / M_PI,
            initial_imu_heading_ * 180.0 / M_PI,
            wrapAngle(last_imu_heading_ - initial_imu_heading_) * 180.0 / M_PI,
            heading * 180.0 / M_PI
        );

        // ===== BEMF velocity (body frame, read at control loop rate ~20Hz) =====

        const foundation::ChassisVelocity body_velocity_raw = kinematics_->estimateState();

        LIBSTP_LOG_DEBUG(
            "FusedOdometry::update [BEMF] dt={:.4f} v_bemf=({:.4f}, {:.4f}) wz={:.4f}",
            dt, body_velocity_raw.vx, body_velocity_raw.vy, body_velocity_raw.wz
        );

        // Rotate BEMF body velocity to world frame using heading
        const auto cos_h = static_cast<float>(std::cos(heading));
        const auto sin_h = static_cast<float>(std::sin(heading));
        const float vx_world = cos_h * body_velocity_raw.vx - sin_h * body_velocity_raw.vy;
        const float vy_world = sin_h * body_velocity_raw.vx + cos_h * body_velocity_raw.vy;

        Eigen::Vector3f v_world(vx_world, vy_world, 0.0f);

        // ===== COMPLEMENTARY FILTER with firmware-integrated accel velocity =====

        if (config_.enable_accel_fusion && fusion_initialized_) {
            float vel[3];
            imu_->getIntegratedVelocity(vel);
            const Eigen::Vector3f v_accel_world(vel[0], vel[1], 0.0f);

            const float alpha = std::clamp(config_.bemf_trust, 0.0f, 1.0f);
            v_world = alpha * v_world + (1.0f - alpha) * v_accel_world;

            LIBSTP_LOG_DEBUG(
                "FusedOdometry::update [FUSE] alpha={:.3f} v_bemf_world=({:.4f}, {:.4f}) "
                "v_accel_world=({:.4f}, {:.4f}) v_fused=({:.4f}, {:.4f})",
                alpha, vx_world, vy_world,
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
            body_velocity_raw.vx, body_velocity_raw.vy,
            v_world.x(), v_world.y(),
            position_.x(), position_.y(),
            heading
        );
    }

    foundation::Pose FusedOdometry::getPose() const
    {
        foundation::Pose pose;
        pose.position = position_;
        pose.heading = static_cast<float>(getHeading());
        return pose;
    }

    DistanceFromOrigin FusedOdometry::getDistanceFromOrigin() const
    {
        // Displacement from origin in world frame
        const Eigen::Vector3f displacement_world = position_ - origin_position_;

        // Get forward/right directions at origin using origin heading
        const auto cos_o = static_cast<float>(std::cos(origin_heading_));
        const auto sin_o = static_cast<float>(std::sin(origin_heading_));
        const Eigen::Vector3f forward_at_origin(cos_o, sin_o, 0.0f);
        const Eigen::Vector3f right_at_origin(-sin_o, cos_o, 0.0f);

        // Project displacement onto forward and right directions
        const double forward = displacement_world.dot(forward_at_origin);
        const double lateral = displacement_world.dot(right_at_origin);

        // Straight-line distance (Euclidean)
        const double straight_line = displacement_world.norm();

        return DistanceFromOrigin{forward, lateral, straight_line};
    }

    double FusedOdometry::getHeading() const
    {
        if (!imu_initialized_) {
            return origin_heading_;
        }

        const double imu_heading_delta = wrapAngle(last_imu_heading_ - initial_imu_heading_);
        return wrapAngle(origin_heading_ + imu_heading_delta);
    }

    double FusedOdometry::getHeadingError(double target_heading_rad) const
    {
        const double current_heading = getHeading();
        return angularError(current_heading, target_heading_rad);
    }

    void FusedOdometry::reset(const foundation::Pose& pose)
    {
        position_ = pose.position;
        origin_position_ = position_;
        origin_heading_ = pose.heading;

        imu_initialized_ = false;
        fusion_initialized_ = false;
        imu_->resetIntegratedVelocity();

        LIBSTP_LOG_TRACE(
            "FusedOdometry::reset to pose pos=({:.3f}, {:.3f}, {:.3f}) heading={:.2f}deg",
            pose.position.x(), pose.position.y(), pose.position.z(),
            origin_heading_ * 180.0 / M_PI
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

        position_ = Eigen::Vector3f::Zero();
        origin_position_ = Eigen::Vector3f::Zero();
        origin_heading_ = 0.0;

        imu_initialized_ = false;
        fusion_initialized_ = false;
        imu_->resetIntegratedVelocity();

        LIBSTP_LOG_TRACE("FusedOdometry::reset to origin");
    }
}
