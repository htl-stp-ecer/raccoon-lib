#include "motion/linear_motion.hpp"
#include "motion/motion_config.hpp"
#include "motion/motion_pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace libstp::motion
{
    static ProfiledPIDController makeLinearProfiledPID(
        const UnifiedMotionPidConfig& pid_config,
        const LinearMotionConfig& linear_config,
        double max_velocity)
    {
        ProfiledPIDController::Config cfg;
        cfg.pid = pid_config.distance;
        cfg.velocity_ff = pid_config.velocity_ff;

        // Select axis-appropriate defaults (strafe has different dynamics than forward)
        const auto& axis_defaults = (linear_config.axis == LinearAxis::Lateral)
            ? pid_config.lateral : pid_config.linear;

        TrapezoidalProfile::Constraints constraints;
        constraints.max_velocity = max_velocity;
        constraints.max_acceleration = axis_defaults.acceleration;
        constraints.max_deceleration = axis_defaults.deceleration;

        return ProfiledPIDController(cfg, constraints);
    }

    static double computeMaxVelocity(const UnifiedMotionPidConfig& pid_config,
                                      const LinearMotionConfig& config)
    {
        const auto& axis = (config.axis == LinearAxis::Lateral)
            ? pid_config.lateral : pid_config.linear;
        double scale = std::clamp(config.speed_scale, 0.01, 1.0);
        return scale * axis.max_velocity;
    }

    LinearMotion::LinearMotion(MotionContext ctx, LinearMotionConfig config)
        : Motion(ctx), cfg_(config)
        , max_velocity_(computeMaxVelocity(ctx_.pid_config, config))
        , profiled_pid_(makeLinearProfiledPID(ctx_.pid_config, config, max_velocity_))
    {
        heading_pid_ = createPidController(ctx_.pid_config, PidType::Heading);
    }

    void LinearMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        unsaturated_cycles_ = 0;
        position_offset_m_ = 0.0;

        prev_primary_position_ = 0.0;
        filtered_velocity_ = 0.0;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        odometry().reset();

        heading_pid_->reset();

        // Reset profiled PID at origin, goal = target distance
        profiled_pid_.reset(0.0);
        profiled_pid_.setGoal(cfg_.distance_m);

        if (cfg_.target_heading_rad.has_value())
        {
            initial_heading_rad_ = cfg_.target_heading_rad.value();
            use_absolute_heading_ = true;
        }
        else
        {
            initial_heading_rad_ = odometry().getHeading();
            use_absolute_heading_ = false;
        }

        LIBSTP_LOG_TRACE("LinearMotion started: axis={}, target={:.3f} m, max_velocity={:.3f} m/s (scale={:.2f}), heading_mode={}",
                    (cfg_.axis == LinearAxis::Forward ? "Forward" : "Lateral"),
                    cfg_.distance_m, max_velocity_, cfg_.speed_scale,
                    (use_absolute_heading_ ? "absolute" : "relative"));
    }

    void LinearMotion::startWarm(double position_offset_m, double initial_velocity_mps)
    {
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        unsaturated_cycles_ = 0;
        position_offset_m_ = position_offset_m;

        prev_primary_position_ = 0.0;
        filtered_velocity_ = initial_velocity_mps;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        // Do NOT reset odometry -- carry position continuously
        heading_pid_->reset();

        // Start profile at position 0 with current velocity, goal = segment distance
        profiled_pid_.reset(0.0, initial_velocity_mps);
        profiled_pid_.setGoal(cfg_.distance_m);

        // Always use absolute heading in warm-start mode (odometry not reset)
        if (cfg_.target_heading_rad.has_value()) {
            initial_heading_rad_ = cfg_.target_heading_rad.value();
        } else {
            initial_heading_rad_ = odometry().getAbsoluteHeading();
        }
        use_absolute_heading_ = true;

        LIBSTP_LOG_TRACE("LinearMotion warm-started: axis={}, target={:.3f} m, offset={:.3f} m, "
                    "initial_vel={:.3f} m/s, max_velocity={:.3f} m/s (scale={:.2f})",
                    (cfg_.axis == LinearAxis::Forward ? "Forward" : "Lateral"),
                    cfg_.distance_m, position_offset_m_, initial_velocity_mps,
                    max_velocity_, cfg_.speed_scale);
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

        // Heading error: absolute mode uses getAbsoluteHeading() so it stays
        // stable across odometry resets; relative mode uses the local heading.
        // Sign convention: positive = need to turn CCW (left) to reach target,
        // matching angularError(current, target) = wrapAngle(target - current).
        const double yaw_error = use_absolute_heading_
            ? std::remainder(initial_heading_rad_ - odometry().getAbsoluteHeading(),
                             2.0 * std::numbers::pi)
            : odometry().getHeadingError(initial_heading_rad_);

        // Extract axis-dependent positions (offset for warm-start segments)
        const bool is_forward = (cfg_.axis == LinearAxis::Forward);
        const double raw_primary = is_forward ? distance_info.forward : distance_info.lateral;
        const double primary_position = raw_primary - position_offset_m_;
        const double cross_track_position = is_forward ? distance_info.lateral : distance_info.forward;

        // Filtered velocity for settling detection
        const double raw_velocity = (primary_position - prev_primary_position_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_primary_position_ = primary_position;

        const double distance_error = cfg_.distance_m - primary_position;
        const double actual_error = distance_error;

        LIBSTP_LOG_TRACE("LinearMotion update: primary={:.3f} m, target={:.3f} m, error={:.3f} m, cross_track={:.3f} m, heading={:.3f} rad, yaw_error={:.3f} rad, filt_vel={:.3f} m/s",
                    primary_position, cfg_.distance_m, distance_error, cross_track_position, current_heading, yaw_error, filtered_velocity_);

        // Check if we've reached the final target distance AND are nearly stopped
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

        // Heading correction: use external override if set, otherwise internal heading PID.
        const double omega_cmd_raw = omega_override_.has_value()
            ? omega_override_.value()
            : heading_pid_->update(yaw_error, dt);

        // Primary axis: Profiled PID generates velocity command
        double primary_cmd_raw = profiled_pid_.calculate(primary_position, dt);
        double primary_cmd = std::clamp(primary_cmd_raw, -max_velocity_, max_velocity_);

        // Apply scaling from previous saturation feedback
        primary_cmd *= speed_scale_;
        // When omega_override is active (line follow), the external PID manages omega directly.
        // Only apply heading_scale for internal heading corrections.
        const double omega_cmd_scaled = omega_override_.has_value()
            ? omega_cmd_raw
            : omega_cmd_raw * heading_scale_;

        LIBSTP_LOG_TRACE("LinearMotion scaled cmd: primary={:.3f}, omega={:.3f} (speed_scale={:.3f}, heading_scale={:.3f})",
                    primary_cmd, omega_cmd_scaled, speed_scale_, heading_scale_);

        // Place commands into correct velocity slots based on axis
        foundation::ChassisVelocity cmd{};
        if (is_forward)
        {
            cmd.vx = primary_cmd;
        }
        else
        {
            cmd.vy = primary_cmd;
        }
        cmd.wz = omega_cmd_scaled;

        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Record telemetry
        const auto sp = profiled_pid_.getSetpoint();
        telemetry_.push_back(LinearMotionTelemetry{
            .time_s = elapsed_time_,
            .dt = dt,
            .target_m = cfg_.distance_m,
            .position_m = primary_position,
            .predicted_m = sp.position,
            .cross_track_m = cross_track_position,
            .distance_error_m = distance_error,
            .actual_error_m = actual_error,
            .yaw_error_rad = yaw_error,
            .filtered_velocity_mps = filtered_velocity_,
            .cmd_vx_mps = cmd.vx,
            .cmd_vy_mps = cmd.vy,
            .cmd_wz_radps = cmd.wz,
            .pid_primary_raw = primary_cmd_raw,
            .pid_cross_raw = 0.0,
            .pid_heading_raw = omega_cmd_raw,
            .setpoint_position_m = sp.position,
            .setpoint_velocity_mps = sp.velocity,
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

            if (speed_scale_ > ctx_.pid_config.saturation_min_scale + kMotionEpsilon)
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

    bool LinearMotion::hasReachedDistance() const
    {
        if (finished_) return true;
        if (!started_) return false;
        // Check distance only, ignore velocity settling
        const auto distance_info = odometry().getDistanceFromOrigin();
        const bool is_forward = (cfg_.axis == LinearAxis::Forward);
        const double raw_primary = is_forward ? distance_info.forward : distance_info.lateral;
        const double primary_position = raw_primary - position_offset_m_;
        const double actual_error = cfg_.distance_m - primary_position;
        return std::abs(actual_error) <= ctx_.pid_config.distance_tolerance_m;
    }

    void LinearMotion::complete()
    {
        finished_ = true;
        if (!suppress_hard_stop_)
        {
            drive().hardStop();
        }
    }
}
