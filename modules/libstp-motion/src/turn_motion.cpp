#include "motion/turn_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace
{
    constexpr double kDegToRad = std::numbers::pi / 180.0;
}

namespace libstp::motion
{
    TurnMotion::TurnMotion(MotionContext ctx, double angle_deg, double max_angular_rate_rad_per_sec)
        : TurnMotion(ctx, TurnConfig{
            .target_angle_rad = angle_deg * kDegToRad,
            .max_angular_rate = max_angular_rate_rad_per_sec
        })
    {
    }

    TurnMotion::TurnMotion(MotionContext ctx, TurnConfig config)
        : Motion(ctx), cfg_(config)
    {
        // Validate and clamp configuration parameters
        if (cfg_.max_angular_rate <= 0.0) cfg_.max_angular_rate = 0.5;
        if (cfg_.angle_tolerance_rad <= 0.0) cfg_.angle_tolerance_rad = 0.01;
        if (cfg_.angle_kp <= 0.0) cfg_.angle_kp = 2.0;
        if (cfg_.min_angular_rate < 0.0) cfg_.min_angular_rate = 0.0;
        cfg_.saturation_derating_factor = std::clamp(cfg_.saturation_derating_factor, 0.1, 0.99);
        cfg_.saturation_min_scale = std::clamp(cfg_.saturation_min_scale, 0.05, 1.0);
        cfg_.saturation_recovery_rate = std::clamp(cfg_.saturation_recovery_rate, 0.0, 0.5);
    }

    void TurnMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        angular_scale_ = 1.0;

        // Reset odometry to establish new origin for this motion
        // This zeros the heading, so our initial heading is 0.0
        odometry().reset();

        // Target heading is simply the desired turn angle (since we reset to 0)
        target_heading_rad_ = cfg_.target_angle_rad;

        LIBSTP_LOG_INFO("TurnMotion started: target_angle = {:.3f} rad ({:.1f} deg), max_angular_rate = {:.3f} rad/s",
                    cfg_.target_angle_rad, cfg_.target_angle_rad / kDegToRad, cfg_.max_angular_rate);
    }

    void TurnMotion::update(double dt)
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

        // Get current heading and compute error
        // getHeadingError() automatically computes the shortest angular path and handles wraparound
        const double current_heading = odometry().getHeading();
        const double heading_error = odometry().getHeadingError(target_heading_rad_);

        LIBSTP_LOG_INFO("TurnMotion update: current_heading = {:.3f} rad ({:.1f} deg), target = {:.3f} rad ({:.1f} deg), error = {:.3f} rad ({:.1f} deg)",
                    current_heading, current_heading / kDegToRad,
                    target_heading_rad_, target_heading_rad_ / kDegToRad,
                    heading_error, heading_error / kDegToRad);

        // Check if we've reached the target angle
        const double error_abs = std::abs(heading_error);
        if (error_abs <= cfg_.angle_tolerance_rad)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            LIBSTP_LOG_INFO("TurnMotion completed: final error = {:.3f} rad ({:.1f} deg)", heading_error, heading_error / kDegToRad);
            return;
        }

        // Compute angular velocity using proportional control
        // Positive error -> need to turn CCW (positive omega)
        // Negative error -> need to turn CW (negative omega)
        double omega_cmd = cfg_.angle_kp * heading_error;

        // Clamp to maximum angular rate
        omega_cmd = std::clamp(omega_cmd, -cfg_.max_angular_rate, cfg_.max_angular_rate);

        // Apply minimum angular rate to prevent stalling
        // Only apply minimum if error is still significant
        if (error_abs > cfg_.angle_tolerance_rad && std::abs(omega_cmd) < cfg_.min_angular_rate)
        {
            const double direction = (heading_error >= 0.0) ? 1.0 : -1.0;
            omega_cmd = direction * cfg_.min_angular_rate;
            LIBSTP_LOG_INFO("TurnMotion: Applying minimum angular rate: omega = {:.3f} rad/s", omega_cmd);
        }

        // Apply scaling from previous saturation feedback
        const double omega_cmd_scaled = omega_cmd * angular_scale_;
        LIBSTP_LOG_INFO("TurnMotion: omega_cmd = {:.3f} rad/s, scaled = {:.3f} rad/s (scale={:.3f})",
                    omega_cmd, omega_cmd_scaled, angular_scale_);

        // Send command: no translation, only rotation
        foundation::ChassisVel cmd{0.0, 0.0, omega_cmd_scaled};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback
        if (motor_cmd.saturated_any)
        {
            const double prev_scale = angular_scale_;
            angular_scale_ = std::max(
                cfg_.saturation_min_scale,
                angular_scale_ * cfg_.saturation_derating_factor);

            LIBSTP_LOG_INFO("TurnMotion: Saturation detected (mask=0x{:X}) -> angular_scale {:.3f}->{:.3f}",
                        motor_cmd.saturation_mask, prev_scale, angular_scale_);
        }
        else
        {
            // Recover scaling when not saturated
            const double prev_scale = angular_scale_;
            angular_scale_ = std::min(1.0, angular_scale_ + cfg_.saturation_recovery_rate);

            if (prev_scale != angular_scale_)
            {
                LIBSTP_LOG_INFO("TurnMotion: Recovery -> angular_scale {:.3f}->{:.3f}",
                            prev_scale, angular_scale_);
            }
        }
    }

    bool TurnMotion::isFinished() const
    {
        return finished_;
    }

    void TurnMotion::complete()
    {
        finished_ = true;
    }
}
