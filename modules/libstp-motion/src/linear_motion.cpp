#include "motion/linear_motion.hpp"
#include "motion/motion_config.hpp"
#include "motion/motion_pid.hpp"
#include "foundation/speed_mode_context.hpp"

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

    void LinearMotion::captureInitialPose()
    {
        const auto pose = odometry().getPose();
        initial_position_m_ = Eigen::Vector2d(static_cast<double>(pose.position.x()),
                                              static_cast<double>(pose.position.y()));
        initial_heading_rad_ = static_cast<double>(pose.heading);
    }

    std::pair<double, double> LinearMotion::projectBodyFrame() const
    {
        const auto pose = odometry().getPose();
        const double dx = static_cast<double>(pose.position.x()) - initial_position_m_.x();
        const double dy = static_cast<double>(pose.position.y()) - initial_position_m_.y();
        // Project onto the *target* heading, not the heading captured at start.
        // The heading PID drives the chassis toward cfg_.target_heading_rad, so
        // the robot's steady-state direction of travel is the target heading. If
        // we projected onto initial_heading_rad_ instead, an absolute-heading
        // motion that begins with a heading error θ_err would measure distance
        // along a direction θ_err off the actual travel: the forward component
        // shrinks by cos(θ_err), so the motion would overshoot by 1/cos(θ_err)
        // before the distance target is satisfied. In relative mode the caller
        // sets target_heading_rad to the current world heading, so this equals
        // the start heading and nothing changes.
        const double cos_h = std::cos(cfg_.target_heading_rad);
        const double sin_h = std::sin(cfg_.target_heading_rad);
        //   forward = pos · (cos θ_t, sin θ_t)
        //   lateral = pos · (-sin θ_t, cos θ_t)   ("right-positive" by libstp-odometry convention)
        const double forward = dx * cos_h + dy * sin_h;
        const double lateral = -dx * sin_h + dy * cos_h;
        return {forward, lateral};
    }

    void LinearMotion::start()
    {
        if (started_) return;
        if (cfg_.has_distance_target)
        {
            foundation::SpeedModeContext::instance().assertBemfAvailable("LinearMotion with distance target");
        }
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

        // No more odometry().reset() — world pose lives in localization and is
        // never disturbed by motion start. We snapshot the start pose instead
        // and project subsequent reads into that body frame.
        captureInitialPose();
        drive().resetVelocityControllers();

        heading_pid_->reset();

        // Reset profiled PID at origin, goal = target distance
        profiled_pid_.reset(0.0);
        profiled_pid_.setGoal(cfg_.distance_m);

        LIBSTP_LOG_TRACE("LinearMotion started: axis={}, target={:.3f} m, max_velocity={:.3f} m/s (scale={:.2f}), target_heading={:.3f} rad",
                    (cfg_.axis == LinearAxis::Forward ? "Forward" : "Lateral"),
                    cfg_.distance_m, max_velocity_, cfg_.speed_scale,
                    cfg_.target_heading_rad);
    }

    void LinearMotion::startWarm(double position_offset_m, double initial_velocity_mps)
    {
        if (cfg_.has_distance_target)
        {
            foundation::SpeedModeContext::instance().assertBemfAvailable("LinearMotion warm-start with distance target");
        }
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        unsaturated_cycles_ = 0;
        // Absolute mode snapshots a fresh segment origin below, so carrying
        // the caller's old odometry offset would double-count prior segments.
        position_offset_m_ = 0.0;

        prev_primary_position_ = 0.0;
        filtered_velocity_ = initial_velocity_mps;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        // Do NOT reset odometry -- carry velocity continuously. Position
        // starts from this segment's captured body-frame origin.
        captureInitialPose();
        heading_pid_->reset();

        // Start profile at position 0 with current velocity, goal = segment distance
        profiled_pid_.reset(0.0, initial_velocity_mps);
        profiled_pid_.setGoal(cfg_.distance_m);

        LIBSTP_LOG_TRACE("LinearMotion warm-started: axis={}, target={:.3f} m, offset={:.3f} m, "
                    "initial_vel={:.3f} m/s, max_velocity={:.3f} m/s (scale={:.2f}), target_heading={:.3f} rad",
                    (cfg_.axis == LinearAxis::Forward ? "Forward" : "Lateral"),
                    cfg_.distance_m, position_offset_m, initial_velocity_mps,
                    max_velocity_, cfg_.speed_scale, cfg_.target_heading_rad);
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

        const auto [body_forward, body_lateral] = projectBodyFrame();
        const double current_heading = odometry().getHeading();

        // Heading error to absolute target: positive = turn CCW (left) to
        // reach target. Matches angularError(current, target) = wrap(target - current).
        const double yaw_error = std::remainder(
            cfg_.target_heading_rad - current_heading,
            2.0 * std::numbers::pi);

        // Extract axis-dependent positions (offset for warm-start segments)
        const bool is_forward = (cfg_.axis == LinearAxis::Forward);
        const double raw_primary = is_forward ? body_forward : body_lateral;
        const double primary_position = raw_primary - position_offset_m_;
        const double cross_track_position = is_forward ? body_lateral : body_forward;

        // Filtered velocity for settling detection
        const double raw_velocity = (primary_position - prev_primary_position_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_primary_position_ = primary_position;

        const double distance_error = cfg_.distance_m - primary_position;
        const double actual_error = distance_error;

        LIBSTP_LOG_TRACE("LinearMotion update: primary={:.3f} m, target={:.3f} m, error={:.3f} m, cross_track={:.3f} m, heading={:.3f} rad, yaw_error={:.3f} rad, filt_vel={:.3f} m/s",
                    primary_position, cfg_.distance_m, distance_error, cross_track_position, current_heading, yaw_error, filtered_velocity_);

        // Check if we've reached the final target distance AND are nearly stopped.
        // Only valid when a distance target was actually set — otherwise an
        // until-only motion (distance_m defaults to 0) would terminate on the
        // first cycle since |0 - position| ≈ 0.
        if (cfg_.has_distance_target &&
            std::abs(actual_error) <= ctx_.pid_config.distance_tolerance_m &&
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

        // Record telemetry. Drop the oldest half once the cap is reached
        // — amortises the O(n) erase across N/2 pushes so the 100 Hz hot
        // path stays roughly O(1) per sample without changing the public
        // std::vector return type of getTelemetry().
        const auto sp = profiled_pid_.getSetpoint();
        if (telemetry_.size() >= kMaxTelemetrySamples)
        {
            const auto keep = static_cast<std::ptrdiff_t>(kMaxTelemetrySamples / 2);
            telemetry_.erase(telemetry_.begin(), telemetry_.end() - keep);
        }
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
        foundation::SpeedModeContext::instance().assertBemfAvailable("LinearMotion::hasReachedDistance");
        if (finished_) return true;
        if (!started_) return false;
        // Check distance only, ignore velocity settling
        const auto [body_forward, body_lateral] = projectBodyFrame();
        const bool is_forward = (cfg_.axis == LinearAxis::Forward);
        const double raw_primary = is_forward ? body_forward : body_lateral;
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
