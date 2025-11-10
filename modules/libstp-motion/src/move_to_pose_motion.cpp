#include "motion/move_to_pose_motion.hpp"
#include "odometry/angle_utils.hpp"

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

        // Reset odometry to establish new origin for this motion
        odometry().reset();

        const double initial_heading = odometry().getHeading();
        const double target_heading = odometry::extractYaw(cfg_.target_pose.orientation);

        SPDLOG_INFO("MoveToPoseMotion started: target_pos = ({:.3f}, {:.3f}), target_heading = {:.3f} rad, "
                    "initial_heading = {:.3f} rad",
                    cfg_.target_pose.position.x(), cfg_.target_pose.position.y(), target_heading,
                    initial_heading);
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

        const auto current_pose = odometry().getPose();

        // ===== COMPUTE POSITION AND HEADING ERRORS =====

        // Position error: target - current (both in world frame from origin)
        const Eigen::Vector3f position_error_world = cfg_.target_pose.position - current_pose.position;
        const Eigen::Vector2f position_error_2d(position_error_world.x(), position_error_world.y());
        const double position_error_magnitude = position_error_2d.norm();

        // Transform position error to body frame using odometry
        const Eigen::Vector3f position_error_body = odometry().transformToBodyFrame(position_error_world);

        // Heading error: compute using target orientation from target pose
        const double target_heading = odometry::extractYaw(cfg_.target_pose.orientation);
        const double heading_error = odometry().getHeadingError(target_heading);

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
                    // Compute desired heading to face the target
                    const double desired_heading = std::atan2(position_error_2d.y(), position_error_2d.x());
                    const double yaw_to_target = odometry().getHeadingError(desired_heading);

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
}
