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
        , imu_initialized_(false)
    {
        if (!imu_) throw std::invalid_argument("imu cannot be null");
        if (!kinematics_) throw std::invalid_argument("kinematics cannot be null");

        // Wait for IMU to receive initial data from coprocessor
        if (!imu_->waitForReady(config_.imu_ready_timeout_ms)) {
            LIBSTP_LOG_WARN("FusedOdometry::ctor - IMU not ready after {}ms timeout, proceeding anyway",
                          config_.imu_ready_timeout_ms);
        }

        // Register callback on IMU accel data arrival (runs in LCM thread)
        if (config_.enable_accel_fusion) {
            imu_->setLinearAccelCallback([this](float ax, float ay, float az) {
                std::lock_guard<std::mutex> lock(accel_mutex_);

                auto now = std::chrono::steady_clock::now();
                if (!accel_callback_active_) {
                    last_accel_time_ = now;
                    accel_callback_active_ = true;
                    return;
                }

                const float dt = std::chrono::duration<float>(now - last_accel_time_).count();
                last_accel_time_ = now;

                if (dt <= 0.0f || dt > 0.1f) return;  // skip gaps

                // Low-pass filter
                const float lpf = std::clamp(config_.accel_lpf_alpha, 0.0f, 1.0f);
                const Eigen::Vector3f raw(ax, ay, az);
                filtered_imu_accel_ = lpf * raw + (1.0f - lpf) * filtered_imu_accel_;

                // Accumulate velocity delta (only XY, robot is 2D)
                accel_delta_v_.x() += filtered_imu_accel_.x() * dt;
                accel_delta_v_.y() += filtered_imu_accel_.y() * dt;
            });
        }

        LIBSTP_LOG_TRACE("FusedOdometry::ctor initialized with IMU and kinematics");
    }

    FusedOdometry::~FusedOdometry()
    {
        if (imu_) {
            imu_->setLinearAccelCallback(nullptr);
        }
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

    Eigen::Vector3f FusedOdometry::getLinearAcceleration() const
    {
        if (!imu_) {
            return Eigen::Vector3f::Zero();
        }

        // Get gravity-compensated linear acceleration from IMU (already computed by coprocessor)
        float linear_accel[3];
        imu_->getLinearAcceleration(linear_accel);
        return Eigen::Vector3f(linear_accel[0], linear_accel[1], linear_accel[2]);
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

        const Eigen::Quaternionf relative_orientation = getRelativeOrientation();
        orientation_ = origin_orientation_ * relative_orientation;
        orientation_.normalize();

        // ===== BEMF velocity (read at control loop rate ~20Hz) =====

        const foundation::ChassisVelocity body_velocity_raw = kinematics_->estimateState();
        const Eigen::Vector3f v_bemf(body_velocity_raw.vx, body_velocity_raw.vy, 0.0f);
        Eigen::Vector3f v_body = v_bemf;

        LIBSTP_LOG_DEBUG(
            "FusedOdometry::update [BEMF] dt={:.4f} v_bemf=({:.4f}, {:.4f}) wz={:.4f}",
            dt, v_bemf.x(), v_bemf.y(), body_velocity_raw.wz
        );

        // ===== COMPLEMENTARY FILTER with accumulated IMU delta-v =====
        //
        // The LCM thread accumulates accel_delta_v_ at IMU rate (~100Hz).
        // Here at control rate (~20Hz) we grab the accumulated delta, build
        // the IMU velocity estimate, and blend with BEMF.

        if (config_.enable_accel_fusion && fusion_initialized_) {
            // Grab and reset accumulated velocity delta from IMU thread
            Eigen::Vector3f delta_v;
            {
                std::lock_guard<std::mutex> lock(accel_mutex_);
                delta_v = accel_delta_v_;
                accel_delta_v_ = Eigen::Vector3f::Zero();
            }

            const Eigen::Vector3f v_imu(
                fused_body_velocity_.x() + delta_v.x(),
                fused_body_velocity_.y() + delta_v.y(),
                0.0f
            );

            const float alpha = std::clamp(config_.bemf_trust, 0.0f, 1.0f);
            v_body = alpha * v_bemf + (1.0f - alpha) * v_imu;

            LIBSTP_LOG_DEBUG(
                "FusedOdometry::update [FUSE] alpha={:.3f} v_bemf=({:.4f}, {:.4f}) "
                "v_imu=({:.4f}, {:.4f}) v_fused=({:.4f}, {:.4f}) "
                "delta_v=({:.4f}, {:.4f})",
                alpha, v_bemf.x(), v_bemf.y(),
                v_imu.x(), v_imu.y(),
                v_body.x(), v_body.y(),
                delta_v.x(), delta_v.y()
            );
        }

        fused_body_velocity_ = v_body;
        fusion_initialized_ = true;

        // Transform to world frame and integrate position
        const Eigen::Vector3f v_world = orientation_ * v_body;
        position_ += v_world * dt_f;

        LIBSTP_LOG_DEBUG(
            "FusedOdometry::update [RESULT] v_body=({:.4f}, {:.4f}) v_world=({:.4f}, {:.4f}) "
            "pos=({:.4f}, {:.4f}) heading={:.4f}",
            v_body.x(), v_body.y(),
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

        // Reset complementary filter state
        fused_body_velocity_ = Eigen::Vector3f::Zero();
        fusion_initialized_ = false;
        {
            std::lock_guard<std::mutex> lock(accel_mutex_);
            accel_delta_v_ = Eigen::Vector3f::Zero();
            filtered_imu_accel_ = Eigen::Vector3f::Zero();
            accel_callback_active_ = false;
        }

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

        // Reset complementary filter state
        fused_body_velocity_ = Eigen::Vector3f::Zero();
        fusion_initialized_ = false;
        {
            std::lock_guard<std::mutex> lock(accel_mutex_);
            accel_delta_v_ = Eigen::Vector3f::Zero();
            filtered_imu_accel_ = Eigen::Vector3f::Zero();
            accel_callback_active_ = false;
        }

        LIBSTP_LOG_TRACE("FusedOdometry::reset to origin");
    }
}
