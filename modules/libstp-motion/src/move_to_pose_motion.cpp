#include "motion/move_to_pose_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "spdlog/spdlog.h"

namespace libstp::motion
{
    MoveToPoseMotion::MoveToPoseMotion(MotionContext ctx, foundation::Pose target_pose,
                                       double max_linear_speed_mps, double max_angular_speed_rps)
        : MoveToPoseMotion(ctx, MoveToPoseConfig{
            .target_pose = target_pose,
            .max_linear_speed_mps = max_linear_speed_mps,
            .max_angular_speed_rps = max_angular_speed_rps
        })
    {
    }

    MoveToPoseMotion::MoveToPoseMotion(MotionContext ctx, MoveToPoseConfig config)
        : Motion(ctx), cfg_(config)
    {
        cfg_.max_linear_speed_mps = std::max(0.01, std::abs(cfg_.max_linear_speed_mps));
        cfg_.max_angular_speed_rps = std::max(0.01, std::abs(cfg_.max_angular_speed_rps));
        cfg_.position_tolerance_m = std::max(0.001, cfg_.position_tolerance_m);
        cfg_.heading_tolerance_rad = std::max(0.001, cfg_.heading_tolerance_rad);
        cfg_.position_kp = std::max(0.1, cfg_.position_kp);
        cfg_.heading_kp = std::max(0.1, cfg_.heading_kp);
    }

    void MoveToPoseMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        reorienting_ = false;

        const auto pose = odometry().getPose();
        reference_orientation_ = pose.orientation.normalized();
        reference_position_ = pose.position;

        const double reference_yaw = computeYaw(reference_orientation_);
        const double target_yaw = computeYaw(cfg_.target_pose.orientation);

        SPDLOG_INFO("MoveToPoseMotion started: target_pos = ({:.3f}, {:.3f}), target_yaw = {:.3f} rad, "
                    "reference_yaw = {:.3f} rad, start_pos = ({:.3f}, {:.3f})",
                    cfg_.target_pose.position.x(), cfg_.target_pose.position.y(), target_yaw,
                    reference_yaw, reference_position_.x(), reference_position_.y());
    }

    void MoveToPoseMotion::update(double dt)
    {
        if (!started_) start();

        if (finished_)
        {
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            if (dt > 0.0)
            {
                [[maybe_unused]] const auto motor_cmd = drive().update(dt);
            }
            return;
        }

        if (dt <= 0.0) return;

        // Update odometry first
        odometry().update(dt);

        const auto pose = odometry().getPose();
        const foundation::ChassisState state = drive().estimateState();

        // ===== COMPUTE POSITION AND HEADING ERRORS =====

        // Position error in world frame (relative to start)
        const Eigen::Vector3f position_error_world = (reference_position_ + cfg_.target_pose.position) - pose.position;
        const Eigen::Vector2f position_error_2d(position_error_world.x(), position_error_world.y());
        const double position_error_magnitude = position_error_2d.norm();

        // Transform position error to body frame
        const Eigen::Quaternionf current_orientation = pose.orientation.normalized();
        const Eigen::Vector3f position_error_body = current_orientation.inverse() * position_error_world;

        // Heading error
        const Eigen::Quaternionf target_orientation_world = reference_orientation_ * cfg_.target_pose.orientation;
        Eigen::Quaternionf target_orientation_world_corrected = target_orientation_world.normalized();

        // Ensure quaternions in same hemisphere
        if (current_orientation.dot(target_orientation_world_corrected) < 0.0f)
        {
            target_orientation_world_corrected.coeffs() *= -1.0f;
        }

        const Eigen::Quaternionf q_error = (target_orientation_world_corrected * current_orientation.conjugate()).normalized();
        const double heading_error = wrapAngle(computeYaw(q_error));

        SPDLOG_INFO("MoveToPoseMotion position_error = ({:.3f}, {:.3f}) mag={:.3f} m, heading_error = {:.3f} rad",
                    position_error_2d.x(), position_error_2d.y(), position_error_magnitude, heading_error);

        // ===== CHECK COMPLETION =====
        if (position_error_magnitude <= cfg_.position_tolerance_m &&
            std::abs(heading_error) <= cfg_.heading_tolerance_rad)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            SPDLOG_INFO("MoveToPoseMotion completed!");
            return;
        }

        // ===== CONTROL STRATEGY =====
        const bool supports_lateral = drive().getKinematics().supportsLateralMotion();

        double vx_cmd = 0.0;
        double vy_cmd = 0.0;
        double omega_cmd = 0.0;

        if (supports_lateral)
        {
            // ===== MECANUM STRATEGY: Direct position control in body frame =====
            vx_cmd = std::clamp(cfg_.position_kp * position_error_body.x(),
                               -cfg_.max_linear_speed_mps, cfg_.max_linear_speed_mps);
            vy_cmd = std::clamp(cfg_.position_kp * position_error_body.y(),
                               -cfg_.max_linear_speed_mps, cfg_.max_linear_speed_mps);
            omega_cmd = std::clamp(cfg_.heading_kp * heading_error,
                                  -cfg_.max_angular_speed_rps, cfg_.max_angular_speed_rps);

            SPDLOG_INFO("MoveToPoseMotion [MECANUM] vx={:.3f}, vy={:.3f}, omega={:.3f}",
                       vx_cmd, vy_cmd, omega_cmd);
        }
        else
        {
            // ===== DIFFERENTIAL STRATEGY: Combined approach =====

            // Decompose error into forward (x) and lateral (y) in body frame
            const double forward_error = position_error_body.x();
            const double lateral_error = position_error_body.y();
            const double lateral_error_abs = std::abs(lateral_error);

            // Check if we need to reorient for large lateral errors
            if (lateral_error_abs > cfg_.lateral_reorient_threshold_m && !reorienting_ &&
                position_error_magnitude > cfg_.position_tolerance_m * 2.0)
            {
                reorienting_ = true;
                SPDLOG_INFO("MoveToPoseMotion [DIFFERENTIAL] Large lateral error ({:.3f} m), entering reorientation mode",
                           lateral_error_abs);
            }

            if (reorienting_)
            {
                // Stop and reorient: point toward target position
                if (position_error_magnitude > 0.01)
                {
                    const float desired_yaw_world = std::atan2(position_error_2d.y(), position_error_2d.x());
                    const double current_yaw_world = computeYaw(current_orientation);
                    double yaw_to_target = wrapAngle(desired_yaw_world - current_yaw_world);

                    omega_cmd = std::clamp(cfg_.heading_kp * yaw_to_target,
                                          -cfg_.max_angular_speed_rps, cfg_.max_angular_speed_rps);
                    vx_cmd = cfg_.max_linear_speed_mps * 0.2;  // Creep forward slowly

                    SPDLOG_INFO("MoveToPoseMotion [DIFFERENTIAL-REORIENT] yaw_to_target={:.3f}, omega={:.3f}",
                               yaw_to_target, omega_cmd);

                    // Exit reorientation when aligned
                    if (std::abs(yaw_to_target) < 0.15 && lateral_error_abs < cfg_.lateral_reorient_threshold_m * 0.7)
                    {
                        reorienting_ = false;
                        SPDLOG_INFO("MoveToPoseMotion [DIFFERENTIAL] Exiting reorientation mode");
                    }
                }
                else
                {
                    reorienting_ = false;
                }
            }
            else
            {
                // Normal mode: drive forward with heading bias for lateral correction
                vx_cmd = std::clamp(cfg_.position_kp * forward_error,
                                   -cfg_.max_linear_speed_mps, cfg_.max_linear_speed_mps);

                // Bias heading based on lateral error
                const double heading_bias = std::atan(cfg_.lateral_heading_bias_gain * lateral_error);
                const double biased_heading_error = heading_error + heading_bias;

                omega_cmd = std::clamp(cfg_.heading_kp * biased_heading_error,
                                      -cfg_.max_angular_speed_rps, cfg_.max_angular_speed_rps);

                SPDLOG_INFO("MoveToPoseMotion [DIFFERENTIAL-BIAS] vx={:.3f}, heading_bias={:.3f}, omega={:.3f}",
                           vx_cmd, heading_bias, omega_cmd);
            }

            // When close to target, prioritize final heading alignment
            if (position_error_magnitude < cfg_.position_tolerance_m * 3.0)
            {
                // Slow down linear motion to allow precise heading alignment
                const double heading_alignment_factor = std::max(0.2, 1.0 - std::abs(heading_error) / 0.5);
                vx_cmd *= heading_alignment_factor;
                SPDLOG_INFO("MoveToPoseMotion [DIFFERENTIAL] Near target, alignment_factor={:.3f}", heading_alignment_factor);
            }

            // Reduce speed during reorientation
            if (reorienting_)
            {
                vx_cmd *= 0.3;
            }
        }

        // Apply speed scaling from saturation feedback
        vx_cmd *= speed_scale_;
        vy_cmd *= speed_scale_;

        // ===== SEND COMMANDS =====
        foundation::ChassisVel cmd{vx_cmd, vy_cmd, omega_cmd};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust speed scaling based on saturation
        if (motor_cmd.saturated_any)
        {
            speed_scale_ = std::max(cfg_.saturation_derating_factor, speed_scale_ * 0.98);
            SPDLOG_INFO("MoveToPoseMotion: Saturation detected, reducing speed_scale to {:.3f}", speed_scale_);
        }
        else
        {
            speed_scale_ = std::min(1.0, speed_scale_ * 1.02);
        }
    }

    bool MoveToPoseMotion::isFinished() const
    {
        return finished_;
    }

    void MoveToPoseMotion::complete()
    {
        finished_ = true;
    }

    double MoveToPoseMotion::computeYaw(const Eigen::Quaternionf& orientation) const
    {
        const Eigen::Quaternionf q = orientation.normalized();
        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double MoveToPoseMotion::wrapAngle(double angle)
    {
        const double two_pi = 2.0 * std::numbers::pi;
        double wrapped = std::fmod(angle + std::numbers::pi, two_pi);
        if (wrapped < 0.0) wrapped += two_pi;
        return wrapped - std::numbers::pi;
    }
}
