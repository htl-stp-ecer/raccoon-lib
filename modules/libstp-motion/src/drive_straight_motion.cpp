#include "motion/drive_straight_motion.hpp"
#include "motion/motion_pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace
{
    constexpr double kCmToM = 0.01;
}

namespace libstp::motion
{
    DriveStraightMotion::DriveStraightMotion(MotionContext ctx, double distance_cm, double max_speed_mps)
        : DriveStraightMotion(ctx, DriveStraightConfig{
            .distance_m = distance_cm * kCmToM,
            .max_speed_mps = max_speed_mps
        })
    {
    }

    DriveStraightMotion::DriveStraightMotion(MotionContext ctx, DriveStraightConfig config)
        : Motion(ctx), cfg_(config)
    {
        cfg_.max_speed_mps = std::abs(cfg_.max_speed_mps);
        if (cfg_.max_speed_mps <= 0.0) cfg_.max_speed_mps = 0.05;
        if (cfg_.distance_tolerance_m <= 0.0) cfg_.distance_tolerance_m = 0.005;
        if (cfg_.distance_kp <= 0.0) cfg_.distance_kp = 1.0;
        if (cfg_.heading_kp < 0.0) cfg_.heading_kp = 0.0;
        if (cfg_.max_heading_rate < 0.0) cfg_.max_heading_rate = 0.0;
        cfg_.saturation_derating_factor = std::clamp(cfg_.saturation_derating_factor, 0.1, 0.99);
        cfg_.saturation_min_speed_scale = std::clamp(cfg_.saturation_min_speed_scale, 0.01, 1.0);
        cfg_.saturation_recovery_rate = std::clamp(cfg_.saturation_recovery_rate, 0.0, 0.5);
        cfg_.saturation_heading_error_rad = std::clamp(cfg_.saturation_heading_error_rad, 0.0, 0.5);
        cfg_.heading_saturation_derating_factor = std::clamp(cfg_.heading_saturation_derating_factor, 0.1, 0.99);
        cfg_.heading_min_scale = std::clamp(cfg_.heading_min_scale, 0.05, 1.0);
        cfg_.heading_recovery_rate = std::clamp(cfg_.heading_recovery_rate, 0.0, 0.5);
        cfg_.heading_recovery_error_rad = std::clamp(cfg_.heading_recovery_error_rad, 0.0, 0.5);

        // Create PID controllers
        MotionPidController::Config distance_pid_cfg;
        distance_pid_cfg.kp = cfg_.distance_kp;
        distance_pid_cfg.ki = cfg_.distance_ki;
        distance_pid_cfg.kd = cfg_.distance_kd;
        distance_pid_cfg.output_min = -cfg_.max_speed_mps;
        distance_pid_cfg.output_max = cfg_.max_speed_mps;
        distance_pid_cfg.integral_max = (cfg_.distance_ki > 0.01) ? (cfg_.max_speed_mps / cfg_.distance_ki) : 10.0;
        distance_pid_cfg.integral_deadband = cfg_.distance_tolerance_m;
        distance_pid_ = std::make_unique<MotionPidController>(distance_pid_cfg);

        MotionPidController::Config heading_pid_cfg;
        heading_pid_cfg.kp = cfg_.heading_kp;
        heading_pid_cfg.ki = cfg_.heading_ki;
        heading_pid_cfg.kd = cfg_.heading_kd;
        heading_pid_cfg.output_min = -cfg_.max_heading_rate;
        heading_pid_cfg.output_max = cfg_.max_heading_rate;
        heading_pid_cfg.integral_max = (cfg_.heading_ki > 0.01) ? (cfg_.max_heading_rate / cfg_.heading_ki) : 10.0;
        heading_pid_cfg.integral_deadband = 0.01;  // ~0.5 degrees
        heading_pid_ = std::make_unique<MotionPidController>(heading_pid_cfg);

        MotionPidController::Config lateral_pid_cfg;
        lateral_pid_cfg.kp = cfg_.lateral_kp;
        lateral_pid_cfg.ki = cfg_.lateral_ki;
        lateral_pid_cfg.kd = cfg_.lateral_kd;
        lateral_pid_cfg.output_min = -cfg_.max_speed_mps;
        lateral_pid_cfg.output_max = cfg_.max_speed_mps;
        lateral_pid_cfg.integral_max = (cfg_.lateral_ki > 0.01) ? (cfg_.max_speed_mps / cfg_.lateral_ki) : 10.0;
        lateral_pid_cfg.integral_deadband = 0.005;  // 5mm
        lateral_pid_ = std::make_unique<MotionPidController>(lateral_pid_cfg);
    }

    void DriveStraightMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        reorienting_ = false;

        // Reset odometry to establish new origin for this motion
        odometry().reset();

        // Reset PID controllers
        distance_pid_->reset();
        heading_pid_->reset();
        lateral_pid_->reset();

        const double initial_heading = odometry().getHeading();
        initial_heading_rad_ = initial_heading;

        LIBSTP_LOG_TRACE("DriveStraightMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s, initial_heading = {:.3f} rad",
                    cfg_.distance_m, cfg_.max_speed_mps, initial_heading);
    }

    void DriveStraightMotion::update(double dt)
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

        // Get all distance and heading info from odometry
        const auto distance_info = odometry().getDistanceFromOrigin();
        const double current_heading = odometry().getHeading();
        const double yaw_error = odometry().getHeadingError(initial_heading_rad_);

        LIBSTP_LOG_TRACE("DriveStraightMotion update: forward = {:.3f} m, lateral = {:.3f} m, heading = {:.3f} rad, yaw_error = {:.3f} rad",
                    distance_info.forward, distance_info.lateral, current_heading, yaw_error);

        // Check if we've reached the target distance
        const double remaining = cfg_.distance_m - distance_info.forward;
        const double remaining_abs = std::abs(remaining);
        LIBSTP_LOG_TRACE("DriveStraightMotion remaining_distance = {:.3f} m", remaining);

        if (remaining_abs <= cfg_.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            return;
        }

        // Lateral error is directly available from odometry
        const double lateral_error_world = distance_info.lateral;
        const double forward_progress = distance_info.forward;

        LIBSTP_LOG_TRACE("DriveStraightMotion lateral_error = {:.3f} m", lateral_error_world);

        // Check if kinematics supports lateral motion
        const bool supports_lateral = drive().getKinematics().supportsLateralMotion();

        // ===== LATERAL CORRECTION STRATEGY =====
        double vy_cmd = 0.0;
        double omega_cmd = 0.0;

        if (supports_lateral)
        {
            // MECANUM STRATEGY: Direct lateral correction using vy
            // Note: lateral_error is positive when drifted right (from odometry)
            // For mecanum, positive vy moves right, so we want negative vy to correct rightward drift
            vy_cmd = -lateral_pid_->update(lateral_error_world, dt);
            vy_cmd = std::clamp(vy_cmd, -cfg_.max_speed_mps * 0.5, cfg_.max_speed_mps * 0.5);
            omega_cmd = heading_pid_->update(yaw_error, dt);
            LIBSTP_LOG_TRACE("DriveStraightMotion [MECANUM] vy_cmd = {:.3f} m/s, omega_cmd = {:.3f} rad/s", vy_cmd, omega_cmd);
        }
        else
        {
            // DIFFERENTIAL STRATEGY: Combined approach (bias + stop-and-reorient)
            const double lateral_error_abs = std::abs(lateral_error_world);

            if (lateral_error_abs > cfg_.lateral_reorient_threshold_m && !reorienting_)
            {
                // Large lateral error: enter reorientation mode
                reorienting_ = true;
                LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL] Large lateral error ({:.3f} m), entering reorientation mode", lateral_error_abs);
            }

            if (reorienting_)
            {
                // Stop and reorient: compute desired heading that points back toward path
                // We want to turn to point at the path line at our current forward progress
                // Lateral error tells us how far perpendicular we are from the path
                // We want a heading that will bring us back to the path
                const double desired_heading_bias = std::atan2(-lateral_error_world, std::max(0.1, cfg_.lateral_reorient_threshold_m));
                const double target_heading = initial_heading_rad_ + desired_heading_bias;
                const double yaw_to_target = odometry().getHeadingError(target_heading);

                omega_cmd = heading_pid_->update(yaw_to_target, dt);
                LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL-REORIENT] yaw_to_target = {:.3f} rad, omega_cmd = {:.3f} rad/s",
                            yaw_to_target, omega_cmd);

                // Exit reorientation when aligned and lateral error is small
                if (std::abs(yaw_to_target) < 0.1 && lateral_error_abs < cfg_.lateral_reorient_threshold_m * 0.7)
                {
                    reorienting_ = false;
                    LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL] Exiting reorientation mode");
                }
            }
            else
            {
                // Normal mode: bias heading slightly to correct lateral drift
                const double heading_bias = std::atan(cfg_.lateral_heading_bias_gain * lateral_error_world);
                const double biased_yaw_error = yaw_error + heading_bias;
                omega_cmd = heading_pid_->update(biased_yaw_error, dt);
                LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL-BIAS] heading_bias = {:.3f} rad, biased_yaw_error = {:.3f} rad, omega_cmd = {:.3f} rad/s",
                            heading_bias, biased_yaw_error, omega_cmd);
            }
        }

        // ===== FORWARD VELOCITY CONTROL =====
        double vx_cmd = distance_pid_->update(remaining, dt);

        if (std::abs(vx_cmd) < 1e-4)
        {
            const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
            vx_cmd = direction * std::min(cfg_.max_speed_mps, 0.05);
        }

        // Reduce forward speed during reorientation (differential only)
        if (reorienting_)
        {
            vx_cmd *= 0.3;  // Slow down to 30% during reorientation
            LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL-REORIENT] Reducing vx_cmd to {:.3f} m/s", vx_cmd);
        }

        // Apply scaling from previous saturation feedback (reduce translation first, then heading if needed)
        vx_cmd *= speed_scale_;
        vy_cmd *= speed_scale_;
        const double omega_cmd_scaled = omega_cmd * heading_scale_;
        LIBSTP_LOG_TRACE(
            "DriveStraightMotion scaled cmd: vx = {:.3f} m/s, vy = {:.3f} m/s, omega = {:.3f} rad/s (speed_scale={:.3f}, heading_scale={:.3f})",
            vx_cmd,
            vy_cmd,
            omega_cmd_scaled,
            speed_scale_,
            heading_scale_);

        // ===== SEND COMMANDS =====
        foundation::ChassisVel cmd{vx_cmd, vy_cmd, omega_cmd_scaled};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback
        const double yaw_error_abs = std::abs(yaw_error);
        if (motor_cmd.saturated_any && yaw_error_abs > cfg_.saturation_heading_error_rad)
        {
            const double prev_speed_scale = speed_scale_;
            const double prev_heading_scale = heading_scale_;

            if (speed_scale_ > cfg_.saturation_min_speed_scale + 1e-6)
            {
                speed_scale_ = std::max(
                    cfg_.saturation_min_speed_scale,
                    speed_scale_ * cfg_.saturation_derating_factor);
            }
            else
            {
                heading_scale_ = std::max(
                    cfg_.heading_min_scale,
                    heading_scale_ * cfg_.heading_saturation_derating_factor);
            }

            LIBSTP_LOG_TRACE(
                "DriveStraightMotion: Saturation detected (mask=0x{:X}, yaw_error={:.3f}) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                motor_cmd.saturation_mask,
                yaw_error,
                prev_speed_scale,
                speed_scale_,
                prev_heading_scale,
                heading_scale_);
        }
        else if (!motor_cmd.saturated_any && yaw_error_abs < cfg_.heading_recovery_error_rad)
        {
            const double prev_speed_scale = speed_scale_;
            const double prev_heading_scale = heading_scale_;

            speed_scale_ = std::min(1.0, speed_scale_ + cfg_.saturation_recovery_rate);
            heading_scale_ = std::min(1.0, heading_scale_ + cfg_.heading_recovery_rate);

            if (prev_speed_scale != speed_scale_ || prev_heading_scale != heading_scale_)
            {
                LIBSTP_LOG_TRACE(
                    "DriveStraightMotion: Recovery -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                    prev_speed_scale,
                    speed_scale_,
                    prev_heading_scale,
                    heading_scale_);
            }
        }
    }

    bool DriveStraightMotion::isFinished() const
    {
        return finished_;
    }

    void DriveStraightMotion::complete()
    {
        finished_ = true;
    }
}
