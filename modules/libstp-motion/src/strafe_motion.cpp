#include "motion/strafe_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "spdlog/spdlog.h"

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
    }

    void StrafeMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;

        // Reset odometry to establish new origin for this motion
        odometry().reset();

        // Store initial heading (should be ~0 after reset) and target distance
        initial_heading_rad_ = odometry().getHeading();
        target_distance_m_ = cfg_.target_distance_m;

        SPDLOG_INFO("StrafeMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s",
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

        SPDLOG_INFO("StrafeMotion update: current_lateral = {:.3f} m, target = {:.3f} m, lateral_error = {:.3f} m, heading_error = {:.3f} rad",
                    current_lateral_m, target_distance_m_, lateral_error, heading_error);

        // Check if we've reached the target distance
        const double error_abs = std::abs(lateral_error);
        if (error_abs <= cfg_.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            SPDLOG_INFO("StrafeMotion completed: final error = {:.3f} m", lateral_error);
            return;
        }

        // Compute lateral velocity using proportional control
        double vy_cmd = cfg_.distance_kp * lateral_error;

        // Clamp to maximum speed
        vy_cmd = std::clamp(vy_cmd, -cfg_.max_speed_mps, cfg_.max_speed_mps);

        // Apply minimum speed to prevent stalling
        if (error_abs > cfg_.distance_tolerance_m && std::abs(vy_cmd) < cfg_.min_speed_mps)
        {
            const double direction = (lateral_error >= 0.0) ? 1.0 : -1.0;
            vy_cmd = direction * cfg_.min_speed_mps;
            SPDLOG_INFO("StrafeMotion: Applying minimum speed: vy = {:.3f} m/s", vy_cmd);
        }

        // Apply scaling from previous saturation feedback
        const double vy_cmd_scaled = vy_cmd * speed_scale_;

        // Compute heading correction to maintain straight strafe
        const double omega_correction = cfg_.heading_kp * heading_error;

        SPDLOG_INFO("StrafeMotion: vy_cmd = {:.3f} m/s, scaled = {:.3f} m/s (scale={:.3f}), omega_correction = {:.3f} rad/s",
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

            SPDLOG_INFO("StrafeMotion: Saturation detected (mask=0x{:X}) -> speed_scale {:.3f}->{:.3f}",
                        motor_cmd.saturation_mask, prev_scale, speed_scale_);
        }
        else
        {
            // Recover scaling when not saturated
            const double prev_scale = speed_scale_;
            speed_scale_ = std::min(1.0, speed_scale_ + cfg_.saturation_recovery_rate);

            if (prev_scale != speed_scale_)
            {
                SPDLOG_INFO("StrafeMotion: Recovery -> speed_scale {:.3f}->{:.3f}",
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
