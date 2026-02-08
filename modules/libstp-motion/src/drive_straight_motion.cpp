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

    DriveStraightMotion::DriveStraightMotion(MotionContext ctx, foundation::Meters distance, foundation::MetersPerSecond max_speed)
        : DriveStraightMotion(ctx, DriveStraightConfig{
            .distance_m = distance.value,
            .max_speed_mps = max_speed.value
        })
    {
    }

    DriveStraightMotion::DriveStraightMotion(MotionContext ctx, DriveStraightConfig config)
        : Motion(ctx), cfg_(config)
    {
        cfg_.max_speed_mps = std::abs(cfg_.max_speed_mps);
        if (cfg_.max_speed_mps <= 0.0) cfg_.max_speed_mps = 0.05;

        // Create PID controllers using factory (eliminates boilerplate)
        distance_pid_ = createPidController(ctx_.pid_config, PidType::Distance);
        heading_pid_ = createPidController(ctx_.pid_config, PidType::Heading);
        lateral_pid_ = createPidController(ctx_.pid_config, PidType::Lateral);
    }

    void DriveStraightMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        reorienting_ = false;
        elapsed_time_ = 0.0;
        unsaturated_cycles_ = 0;

        // Reset odometry to establish new origin for this motion
        odometry().reset();

        // Reset PID controllers
        distance_pid_->reset();
        heading_pid_->reset();
        lateral_pid_->reset();

        const double initial_heading = odometry().getHeading();
        initial_heading_rad_ = initial_heading;

        // Create trapezoidal profile for smooth motion
        TrapezoidalProfile::State initial{0.0, 0.0};  // Start at position=0, velocity=0
        TrapezoidalProfile::Constraints constraints{
            .max_velocity = cfg_.max_speed_mps,
            .max_acceleration = cfg_.max_acceleration_mps2
        };
        profile_ = std::make_unique<TrapezoidalProfile>(initial, cfg_.distance_m, constraints);

        LIBSTP_LOG_TRACE("DriveStraightMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s, initial_heading = {:.3f} rad, profile_time = {:.3f} s",
                    cfg_.distance_m, cfg_.max_speed_mps, initial_heading, profile_->getTotalTime());
    }

    void DriveStraightMotion::update(double dt)
    {
        if (!started_) start();

        if (finished_)
        {
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            if (dt > 0.0)
            {
                [[maybe_unused]] const auto motor_cmd = drive().update(dt);
            }
            return;
        }

        if (dt <= 0.0)
        {
            LIBSTP_LOG_WARN("DriveStraightMotion::update called with invalid dt={:.6f}s (must be > 0)", dt);
            return;
        }

        // Update elapsed time
        elapsed_time_ += dt;

        // Update odometry first
        odometry().update(dt);

        // Get setpoint from trapezoidal profile
        const auto setpoint = profile_->getSetpoint(elapsed_time_);

        // Get all distance and heading info from odometry
        const auto distance_info = odometry().getDistanceFromOrigin();
        const double current_heading = odometry().getHeading();
        const double yaw_error = odometry().getHeadingError(initial_heading_rad_);

        // Compute distance error against the ramped setpoint (not the final target!)
        const double distance_error = setpoint.position - distance_info.forward;
        const double error_to_final_target = cfg_.distance_m - distance_info.forward;

        LIBSTP_LOG_TRACE("DriveStraightMotion update: forward = {:.3f} m, setpoint = {:.3f} m, target = {:.3f} m, lateral = {:.3f} m, heading = {:.3f} rad, yaw_error = {:.3f} rad",
                    distance_info.forward, setpoint.position, cfg_.distance_m, distance_info.lateral, current_heading, yaw_error);

        // Check if we've reached the final target distance
        const double remaining = error_to_final_target;
        const double remaining_abs = std::abs(remaining);
        LIBSTP_LOG_TRACE("DriveStraightMotion remaining_distance = {:.3f} m", remaining);

        if (remaining_abs <= ctx_.pid_config.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
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

            if (lateral_error_abs > ctx_.pid_config.lateral_reorient_threshold_m && !reorienting_)
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
                const double desired_heading_bias = std::atan2(-lateral_error_world, std::max(0.1, ctx_.pid_config.lateral_reorient_threshold_m));
                const double target_heading = initial_heading_rad_ + desired_heading_bias;
                const double yaw_to_target = odometry().getHeadingError(target_heading);

                omega_cmd = heading_pid_->update(yaw_to_target, dt);
                LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL-REORIENT] yaw_to_target = {:.3f} rad, omega_cmd = {:.3f} rad/s",
                            yaw_to_target, omega_cmd);

                // Exit reorientation when aligned and lateral error is small
                if (std::abs(yaw_to_target) < 0.1 && lateral_error_abs < ctx_.pid_config.lateral_reorient_threshold_m * 0.7)
                {
                    reorienting_ = false;
                    LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL] Exiting reorientation mode");
                }
            }
            else
            {
                // Normal mode: bias heading slightly to correct lateral drift
                const double heading_bias = std::atan(ctx_.pid_config.lateral_heading_bias_gain * lateral_error_world);
                const double biased_yaw_error = yaw_error + heading_bias;
                omega_cmd = heading_pid_->update(biased_yaw_error, dt);
                LIBSTP_LOG_TRACE("DriveStraightMotion [DIFFERENTIAL-BIAS] heading_bias = {:.3f} rad, biased_yaw_error = {:.3f} rad, omega_cmd = {:.3f} rad/s",
                            heading_bias, biased_yaw_error, omega_cmd);
            }
        }

        // ===== FORWARD VELOCITY CONTROL =====
        // Use the distance error against the setpoint (not the final target)
        double vx_cmd = distance_pid_->update(distance_error, dt);

        if (std::abs(vx_cmd) < 1e-4)
        {
            const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
            vx_cmd = direction * std::min(cfg_.max_speed_mps, ctx_.pid_config.min_speed_mps);
        }

        // Reduce forward speed during reorientation (differential only)
        if (reorienting_)
        {
            vx_cmd *= ctx_.pid_config.reorientation_speed_factor;
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
        foundation::ChassisVelocity cmd{vx_cmd, vy_cmd, omega_cmd_scaled};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback with hysteresis to prevent oscillation
        const double yaw_error_abs = std::abs(yaw_error);
        if (motor_cmd.saturated_any && yaw_error_abs > ctx_.pid_config.heading_saturation_error_rad)
        {
            // Reset hysteresis counter on saturation
            unsaturated_cycles_ = 0;

            const double prev_speed_scale = speed_scale_;
            const double prev_heading_scale = heading_scale_;

            if (speed_scale_ > ctx_.pid_config.saturation_min_scale + 1e-6)
            {
                speed_scale_ = std::max(
                    ctx_.pid_config.saturation_min_scale,
                    speed_scale_ * ctx_.pid_config.saturation_derating_factor);
            }
            else
            {
                heading_scale_ = std::max(
                    ctx_.pid_config.heading_min_scale,
                    heading_scale_ * ctx_.pid_config.heading_saturation_derating_factor);
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
        else if (!motor_cmd.saturated_any && yaw_error_abs < ctx_.pid_config.heading_recovery_error_rad)
        {
            // Hysteresis: only recover after sustained period without saturation
            ++unsaturated_cycles_;

            // Only recover if we've been unsaturated long enough AND we're below recovery threshold
            const bool can_recover = unsaturated_cycles_ >= ctx_.pid_config.saturation_hold_cycles;
            const bool needs_recovery = speed_scale_ < ctx_.pid_config.saturation_recovery_threshold ||
                                        heading_scale_ < ctx_.pid_config.saturation_recovery_threshold;

            if (can_recover && needs_recovery)
            {
                const double prev_speed_scale = speed_scale_;
                const double prev_heading_scale = heading_scale_;

                speed_scale_ = std::min(1.0, speed_scale_ + ctx_.pid_config.saturation_recovery_rate);
                heading_scale_ = std::min(1.0, heading_scale_ + ctx_.pid_config.heading_recovery_rate);

                if (prev_speed_scale != speed_scale_ || prev_heading_scale != heading_scale_)
                {
                    LIBSTP_LOG_TRACE(
                        "DriveStraightMotion: Recovery (after {} cycles) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                        unsaturated_cycles_,
                        prev_speed_scale,
                        speed_scale_,
                        prev_heading_scale,
                        heading_scale_);
                }
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
