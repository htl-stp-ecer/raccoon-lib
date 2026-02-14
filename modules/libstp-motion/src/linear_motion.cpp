#include "motion/linear_motion.hpp"
#include "motion/motion_pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace libstp::motion
{
    LinearMotion::LinearMotion(MotionContext ctx, LinearMotionConfig config)
        : Motion(ctx), cfg_(config)
    {
        cfg_.max_speed_mps = std::abs(cfg_.max_speed_mps);
        if (cfg_.max_speed_mps <= 0.0) cfg_.max_speed_mps = 0.05;

        heading_pid_ = createPidController(ctx_.pid_config, PidType::Heading);
        cross_track_pid_ = createPidController(ctx_.pid_config, PidType::Lateral);
    }

    void LinearMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        reorienting_ = false;
        unsaturated_cycles_ = 0;

        prev_primary_position_ = 0.0;
        filtered_velocity_ = 0.0;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        odometry().reset();

        heading_pid_->reset();
        cross_track_pid_->reset();

        initial_heading_rad_ = odometry().getHeading();

        LIBSTP_LOG_TRACE("LinearMotion started: axis={}, target={:.3f} m, max_speed={:.3f} m/s",
                    (cfg_.axis == LinearAxis::Forward ? "Forward" : "Lateral"),
                    cfg_.distance_m, cfg_.max_speed_mps);
    }

    void LinearMotion::update(double dt)
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
            LIBSTP_LOG_WARN("LinearMotion::update called with invalid dt={:.6f}s (must be > 0)", dt);
            return;
        }

        elapsed_time_ += dt;

        odometry().update(dt);

        const auto distance_info = odometry().getDistanceFromOrigin();
        const double current_heading = odometry().getHeading();
        const double yaw_error = odometry().getHeadingError(initial_heading_rad_);

        // Extract axis-dependent positions
        const bool is_forward = (cfg_.axis == LinearAxis::Forward);
        const double primary_position = is_forward ? distance_info.forward : distance_info.lateral;
        const double cross_track_position = is_forward ? distance_info.lateral : distance_info.forward;

        // Filtered velocity for settling detection
        const double raw_velocity = (primary_position - prev_primary_position_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_primary_position_ = primary_position;

        // Use actual position for distance error (no Smith predictor).
        // Prediction-based control causes oscillation with the ~300ms response lag:
        // noisy velocity estimate → noisy prediction → premature braking → oscillation.
        // Instead, velocity damping (distance_kd) compensates for lag directly.
        const double distance_error = cfg_.distance_m - primary_position;
        const double actual_error = distance_error;

        LIBSTP_LOG_TRACE("LinearMotion update: primary={:.3f} m, target={:.3f} m, error={:.3f} m, cross_track={:.3f} m, heading={:.3f} rad, yaw_error={:.3f} rad, filt_vel={:.3f} m/s",
                    primary_position, cfg_.distance_m, distance_error, cross_track_position, current_heading, yaw_error, filtered_velocity_);

        // Check if we've reached the final target distance AND are nearly stopped
        // Use actual position for completion (not predicted), so we don't stop short
        if (std::abs(actual_error) <= ctx_.pid_config.distance_tolerance_m &&
            std::abs(filtered_velocity_) < kSettlingVelocity)
        {
            LIBSTP_LOG_TRACE("LinearMotion completed: primary={:.3f} m, error={:.4f} m, filt_vel={:.4f} m/s",
                        primary_position, actual_error, filtered_velocity_);
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            return;
        }

        // Cross-track correction
        const double cross_track_error = cross_track_position;
        const bool supports_lateral = drive().getKinematics().supportsLateralMotion();

        double cross_cmd_raw = 0.0;
        double omega_cmd_raw = 0.0;

        if (supports_lateral)
        {
            // MECANUM: direct cross-track correction
            cross_cmd_raw = -cross_track_pid_->update(cross_track_error, dt);
            cross_cmd_raw = std::clamp(cross_cmd_raw, -cfg_.max_speed_mps * 0.5, cfg_.max_speed_mps * 0.5);
            omega_cmd_raw = heading_pid_->update(yaw_error, dt);
        }
        else
        {
            // DIFFERENTIAL: heading bias + stop-and-reorient (only valid for forward axis)
            const double cross_track_error_abs = std::abs(cross_track_error);

            if (cross_track_error_abs > ctx_.pid_config.lateral_reorient_threshold_m && !reorienting_)
            {
                reorienting_ = true;
                LIBSTP_LOG_TRACE("LinearMotion [DIFFERENTIAL] Large cross-track error ({:.3f} m), entering reorientation mode", cross_track_error_abs);
            }

            if (reorienting_)
            {
                const double desired_heading_bias = std::atan2(-cross_track_error, std::max(0.1, ctx_.pid_config.lateral_reorient_threshold_m));
                const double target_heading = initial_heading_rad_ + desired_heading_bias;
                const double yaw_to_target = odometry().getHeadingError(target_heading);

                omega_cmd_raw = heading_pid_->update(yaw_to_target, dt);

                if (std::abs(yaw_to_target) < 0.1 && cross_track_error_abs < ctx_.pid_config.lateral_reorient_threshold_m * 0.7)
                {
                    reorienting_ = false;
                    LIBSTP_LOG_TRACE("LinearMotion [DIFFERENTIAL] Exiting reorientation mode");
                }
            }
            else
            {
                const double heading_bias = std::atan(ctx_.pid_config.lateral_heading_bias_gain * cross_track_error);
                const double biased_yaw_error = yaw_error + heading_bias;
                omega_cmd_raw = heading_pid_->update(biased_yaw_error, dt);
            }
        }

        // Primary axis velocity control: PD on position error + output clamp.
        //
        // Same approach that works for turn motion: kP * error provides natural
        // deceleration near the target, while max_speed clamp creates an implicit
        // trapezoidal velocity profile. Velocity damping (kD) compensates for the
        // ~300ms response lag by providing braking force proportional to speed.
        //
        // The previous braking-distance formula v_brake = sqrt(2*decel*d) was too
        // conservative (0.245 m/s for 30cm), and the min_speed floor overrode the
        // D-term's braking commands, causing late surges and massive overshoot.
        const double sign_d = (distance_error >= 0.0) ? 1.0 : -1.0;
        double primary_cmd_raw = ctx_.pid_config.distance_kp * distance_error;
        double primary_cmd = std::clamp(primary_cmd_raw, -cfg_.max_speed_mps, cfg_.max_speed_mps);

        // Velocity damping: D-term on distance error (d/dt(target - pos) = -velocity).
        // Damps approach velocity to prevent overshoot through the response lag.
        primary_cmd -= ctx_.pid_config.distance_kd * filtered_velocity_;

        // Clamp after damping to stay within speed limits
        primary_cmd = std::clamp(primary_cmd, -cfg_.max_speed_mps, cfg_.max_speed_mps);

        // No min_speed floor — it fights the D-term braking and causes overshoot.
        // With PD control the proportional term naturally provides enough force
        // to overcome static friction when far from target.

        // Reduce primary speed during reorientation (differential only)
        if (reorienting_)
        {
            primary_cmd *= ctx_.pid_config.reorientation_speed_factor;
        }

        // Apply scaling from previous saturation feedback
        primary_cmd *= speed_scale_;
        double cross_cmd = cross_cmd_raw * speed_scale_;
        const double omega_cmd_scaled = omega_cmd_raw * heading_scale_;

        LIBSTP_LOG_TRACE("LinearMotion scaled cmd: primary={:.3f}, cross={:.3f}, omega={:.3f} (speed_scale={:.3f}, heading_scale={:.3f})",
                    primary_cmd, cross_cmd, omega_cmd_scaled, speed_scale_, heading_scale_);

        // Place commands into correct velocity slots based on axis
        foundation::ChassisVelocity cmd{};
        if (is_forward)
        {
            cmd.vx = primary_cmd;
            cmd.vy = cross_cmd;
        }
        else
        {
            cmd.vy = primary_cmd;
            cmd.vx = cross_cmd;
        }
        cmd.wz = omega_cmd_scaled;

        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Record telemetry
        telemetry_.push_back(LinearMotionTelemetry{
            .time_s = elapsed_time_,
            .dt = dt,
            .target_m = cfg_.distance_m,
            .position_m = primary_position,
            .predicted_m = primary_position,  // No prediction; kept for telemetry compat
            .cross_track_m = cross_track_position,
            .distance_error_m = distance_error,
            .actual_error_m = actual_error,
            .yaw_error_rad = yaw_error,
            .filtered_velocity_mps = filtered_velocity_,
            .cmd_vx_mps = cmd.vx,
            .cmd_vy_mps = cmd.vy,
            .cmd_wz_radps = cmd.wz,
            .pid_primary_raw = primary_cmd_raw,
            .pid_cross_raw = cross_cmd_raw,
            .pid_heading_raw = omega_cmd_raw,
            .heading_rad = current_heading,
            .speed_scale = speed_scale_,
            .heading_scale = heading_scale_,
            .saturated = motor_cmd.saturated_any,
        });

        // Adjust scaling based on saturation feedback with hysteresis
        const double yaw_error_abs = std::abs(yaw_error);
        if (motor_cmd.saturated_any && yaw_error_abs > ctx_.pid_config.heading_saturation_error_rad)
        {
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
                "LinearMotion: Saturation detected (mask=0x{:X}, yaw_error={:.3f}) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                motor_cmd.saturation_mask, yaw_error,
                prev_speed_scale, speed_scale_,
                prev_heading_scale, heading_scale_);
        }
        else if (!motor_cmd.saturated_any && yaw_error_abs < ctx_.pid_config.heading_recovery_error_rad)
        {
            ++unsaturated_cycles_;

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
                        "LinearMotion: Recovery (after {} cycles) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                        unsaturated_cycles_,
                        prev_speed_scale, speed_scale_,
                        prev_heading_scale, heading_scale_);
                }
            }
        }
    }

    bool LinearMotion::isFinished() const
    {
        return finished_;
    }

    void LinearMotion::complete()
    {
        finished_ = true;
        drive().hardStop();
    }
}
