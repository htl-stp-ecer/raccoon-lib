#include "motion/strafe_motion.hpp"
#include "motion/motion_pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace libstp::motion
{
    StrafeMotion::StrafeMotion(MotionContext ctx, double distance_m, double max_speed_mps)
        : StrafeMotion(ctx, StrafeConfig{
            .target_distance_m = distance_m,
            .max_speed_mps = max_speed_mps
        })
    {
    }

    StrafeMotion::StrafeMotion(MotionContext ctx, StrafeConfig config)
        : Motion(ctx), cfg_(config)
    {
        // Validate and clamp configuration parameters
        if (cfg_.max_speed_mps <= 0.0) cfg_.max_speed_mps = 0.3;
        if (cfg_.distance_tolerance_m <= 0.0) cfg_.distance_tolerance_m = 0.01;
        if (cfg_.distance_kp <= 0.0) cfg_.distance_kp = 2.0;
        if (cfg_.heading_kp <= 0.0) cfg_.heading_kp = 3.0;
        if (cfg_.min_speed_mps < 0.0) cfg_.min_speed_mps = 0.0;
        cfg_.saturation_derating_factor = std::clamp(cfg_.saturation_derating_factor, 0.1, 0.99);
        cfg_.saturation_min_scale = std::clamp(cfg_.saturation_min_scale, 0.05, 1.0);
        cfg_.saturation_recovery_rate = std::clamp(cfg_.saturation_recovery_rate, 0.0, 0.5);

        // Create PID controllers
        MotionPidController::Config lateral_pid_cfg;
        lateral_pid_cfg.kp = cfg_.distance_kp;
        lateral_pid_cfg.ki = cfg_.distance_ki;
        lateral_pid_cfg.kd = cfg_.distance_kd;
        lateral_pid_cfg.output_min = -cfg_.max_speed_mps;
        lateral_pid_cfg.output_max = cfg_.max_speed_mps;
        lateral_pid_cfg.integral_max = (cfg_.distance_ki > 0.01) ? (cfg_.max_speed_mps / cfg_.distance_ki) : 10.0;
        lateral_pid_cfg.integral_deadband = cfg_.distance_tolerance_m;
        lateral_pid_ = std::make_unique<MotionPidController>(lateral_pid_cfg);

        MotionPidController::Config heading_pid_cfg;
        heading_pid_cfg.kp = cfg_.heading_kp;
        heading_pid_cfg.ki = cfg_.heading_ki;
        heading_pid_cfg.kd = cfg_.heading_kd;
        heading_pid_cfg.output_min = -3.0;  // Max heading correction rate
        heading_pid_cfg.output_max = 3.0;
        heading_pid_cfg.integral_max = (cfg_.heading_ki > 0.01) ? (3.0 / cfg_.heading_ki) : 10.0;
        heading_pid_cfg.integral_deadband = 0.01;  // ~0.5 degrees
        heading_pid_ = std::make_unique<MotionPidController>(heading_pid_cfg);
    }

    void StrafeMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;

        // Reset odometry to establish new origin for this motion
        odometry().reset();

        // Reset PID controllers
        lateral_pid_->reset();
        heading_pid_->reset();

        // Store initial heading (should be ~0 after reset) and target distance
        initial_heading_rad_ = odometry().getHeading();
        target_distance_m_ = cfg_.target_distance_m;

        LIBSTP_LOG_TRACE("StrafeMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s",
                    cfg_.target_distance_m, cfg_.max_speed_mps);
    }

    void StrafeMotion::update(double dt)
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

        if (dt <= 0.0)
        {
            return;
        }

        // Update odometry first
        odometry().update(dt);

        // Get distance from origin - lateral is perpendicular to initial forward direction
        // Note: positive lateral (from odometry) = moved right, negative = moved left
        const auto distance_info = odometry().getDistanceFromOrigin();
        const double current_lateral_m = distance_info.lateral;  // Use as-is: positive=right, negative=left
        const double current_heading = odometry().getHeading();

        // Compute lateral distance error
        // Positive error → need to move right (positive vy)
        // Negative error → need to move left (negative vy)
        const double lateral_error = target_distance_m_ - current_lateral_m;

        // Compute heading error (maintain initial heading)
        const double heading_error = odometry().getHeadingError(initial_heading_rad_);

        LIBSTP_LOG_TRACE("StrafeMotion update: current_lateral = {:.3f} m, target = {:.3f} m, lateral_error = {:.3f} m, heading_error = {:.3f} rad",
                    current_lateral_m, target_distance_m_, lateral_error, heading_error);

        // Check if we've reached the target distance
        const double error_abs = std::abs(lateral_error);
        if (error_abs <= cfg_.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            LIBSTP_LOG_TRACE("StrafeMotion completed: final error = {:.3f} m", lateral_error);
            return;
        }

        // Compute lateral velocity using PID control
        double vy_cmd = lateral_pid_->update(lateral_error, dt);

        // Apply minimum speed to prevent stalling
        if (error_abs > cfg_.distance_tolerance_m && std::abs(vy_cmd) < cfg_.min_speed_mps)
        {
            const double direction = (lateral_error >= 0.0) ? 1.0 : -1.0;
            vy_cmd = direction * cfg_.min_speed_mps;
            LIBSTP_LOG_TRACE("StrafeMotion: Applying minimum speed: vy = {:.3f} m/s", vy_cmd);
        }

        // Apply scaling from previous saturation feedback
        const double vy_cmd_scaled = vy_cmd * speed_scale_;

        // Compute heading correction to maintain straight strafe
        const double omega_correction = heading_pid_->update(heading_error, dt);

        LIBSTP_LOG_TRACE("StrafeMotion: vy_cmd = {:.3f} m/s, scaled = {:.3f} m/s (scale={:.3f}), omega_correction = {:.3f} rad/s",
                    vy_cmd, vy_cmd_scaled, speed_scale_, omega_correction);

        // Send command: no forward motion, lateral strafe, with heading correction
        foundation::ChassisVel cmd{0.0, vy_cmd_scaled, omega_correction};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback
        if (motor_cmd.saturated_any)
        {
            const double prev_scale = speed_scale_;
            speed_scale_ = std::max(
                cfg_.saturation_min_scale,
                speed_scale_ * cfg_.saturation_derating_factor);

            LIBSTP_LOG_TRACE("StrafeMotion: Saturation detected (mask=0x{:X}) -> speed_scale {:.3f}->{:.3f}",
                        motor_cmd.saturation_mask, prev_scale, speed_scale_);
        }
        else
        {
            // Recover scaling when not saturated
            const double prev_scale = speed_scale_;
            speed_scale_ = std::min(1.0, speed_scale_ + cfg_.saturation_recovery_rate);

            if (prev_scale != speed_scale_)
            {
                LIBSTP_LOG_TRACE("StrafeMotion: Recovery -> speed_scale {:.3f}->{:.3f}",
                            prev_scale, speed_scale_);
            }
        }
    }

    bool StrafeMotion::isFinished() const
    {
        return finished_;
    }

    void StrafeMotion::complete()
    {
        finished_ = true;
    }
}
