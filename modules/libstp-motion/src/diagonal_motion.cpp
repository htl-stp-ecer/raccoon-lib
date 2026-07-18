#include "motion/diagonal_motion.hpp"
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
    static double computeDiagonalMaxVelocity(const UnifiedMotionPidConfig& pid_config,
                                              const DiagonalMotionConfig& config)
    {
        double scale = std::clamp(config.speed_scale, 0.01, 1.0);
        return scale * pid_config.linear.max_velocity;
    }

    static ProfiledPIDController makeDiagonalProfiledPID(
        const UnifiedMotionPidConfig& pid_config,
        double max_velocity)
    {
        ProfiledPIDController::Config cfg;
        cfg.pid = pid_config.distance;
        cfg.velocity_ff = pid_config.velocity_ff;

        TrapezoidalProfile::Constraints constraints;
        constraints.max_velocity = max_velocity;
        constraints.max_acceleration = pid_config.linear.acceleration;
        constraints.max_deceleration = pid_config.linear.deceleration;

        return ProfiledPIDController(cfg, constraints);
    }

    DiagonalMotion::DiagonalMotion(MotionContext ctx, DiagonalMotionConfig config)
        : Motion(ctx), cfg_(config)
        , max_velocity_(computeDiagonalMaxVelocity(ctx_.pid_config, config))
        , profiled_pid_(makeDiagonalProfiledPID(ctx_.pid_config, max_velocity_))
    {
        // Precompute trig for rotated coordinate frame
        cos_angle_ = std::cos(cfg_.angle_rad);
        sin_angle_ = std::sin(cfg_.angle_rad);

        heading_pid_ = createPidController(ctx_.pid_config, PidType::Heading);
    }

    void DiagonalMotion::start()
    {
        if (started_) return;
        if (cfg_.has_distance_target)
        {
            foundation::SpeedModeContext::instance().assertBemfAvailable("DiagonalMotion with distance target");
        }
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        unsaturated_cycles_ = 0;

        prev_primary_position_ = 0.0;
        filtered_velocity_ = 0.0;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        // Snapshot start pose; do not reset odometry.
        {
            const auto pose = odometry().getPose();
            initial_position_m_ = Eigen::Vector2d(static_cast<double>(pose.position.x()),
                                                  static_cast<double>(pose.position.y()));
            initial_heading_rad_ = static_cast<double>(pose.heading);
        }
        drive().resetVelocityControllers();

        heading_pid_->reset();

        // Reset profiled PID at origin, goal = target distance
        profiled_pid_.reset(0.0);
        profiled_pid_.setGoal(cfg_.distance_m);

        LIBSTP_LOG_TRACE("DiagonalMotion started: angle={:.3f} rad, target={:.3f} m, max_velocity={:.3f} m/s (scale={:.2f})",
                    cfg_.angle_rad, cfg_.distance_m, max_velocity_, cfg_.speed_scale);
    }

    void DiagonalMotion::update(double dt)
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
            LIBSTP_LOG_WARN("DiagonalMotion::update called with invalid dt={:.6f}s (must be > 0)", dt);
            return;
        }

        elapsed_time_ += dt;

        odometry().update(dt);

        // Project current world-frame pose into the body frame captured at start.
        const auto pose = odometry().getPose();
        const double dx = static_cast<double>(pose.position.x()) - initial_position_m_.x();
        const double dy = static_cast<double>(pose.position.y()) - initial_position_m_.y();
        const double cos_h = std::cos(initial_heading_rad_);
        const double sin_h = std::sin(initial_heading_rad_);
        const double body_forward = dx * cos_h + dy * sin_h;
        const double body_lateral = -dx * sin_h + dy * cos_h;

        // Close the heading loop on getHeading() — the SAME frame as
        // cfg_.target_heading_rad (set from getPose().heading, either via
        // get_world_heading_rad or HeadingReferenceService) and as
        // initial_heading_rad_ used for the cross-track projection above.
        // Using getAbsoluteHeading() here mixed the raw-IMU frame (never reset)
        // with the odometry/heading-reference frame, injecting a phantom yaw
        // error equal to their offset whenever odometry had been reset/resynced
        // — the controller then slammed the chassis into a spurious turn.
        // LinearMotion/TurnMotion already close on getHeading(); match them.
        const double current_heading = odometry().getHeading();
        // Heading error to absolute target.
        const double yaw_error = std::remainder(
            cfg_.target_heading_rad - current_heading,
            2.0 * std::numbers::pi);

        // Project body-frame displacement onto rotated travel frame
        // Primary axis = along travel direction, cross-track = perpendicular
        const double primary_position = body_forward * cos_angle_ + body_lateral * sin_angle_;
        const double cross_track_position = -body_forward * sin_angle_ + body_lateral * cos_angle_;

        // Filtered velocity for settling detection
        const double raw_velocity = (primary_position - prev_primary_position_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_primary_position_ = primary_position;

        const double distance_error = cfg_.distance_m - primary_position;
        const double actual_error = distance_error;

        LIBSTP_LOG_TRACE("DiagonalMotion update: primary={:.3f} m, target={:.3f} m, error={:.3f} m, cross_track={:.3f} m, heading={:.3f} rad, yaw_error={:.3f} rad, filt_vel={:.3f} m/s",
                    primary_position, cfg_.distance_m, distance_error, cross_track_position, current_heading, yaw_error, filtered_velocity_);

        // Check if we've reached the final target distance AND are nearly stopped.
        // Only valid when a distance target was set — otherwise an until-only
        // strafe (distance_m defaults to 0) terminates on cycle 0.
        if (cfg_.has_distance_target &&
            std::abs(actual_error) <= ctx_.pid_config.distance_tolerance_m &&
            std::abs(filtered_velocity_) < kSettlingVelocity)
        {
            LIBSTP_LOG_TRACE("DiagonalMotion completed: primary={:.3f} m, error={:.4f} m, filt_vel={:.4f} m/s",
                        primary_position, actual_error, filtered_velocity_);
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            return;
        }

        // Heading correction only — no cross-track correction.
        const double omega_cmd_raw = heading_pid_->update(yaw_error, dt);

        // Primary axis: Profiled PID generates velocity command
        double primary_cmd_raw = profiled_pid_.calculate(primary_position, dt);
        double primary_cmd = std::clamp(primary_cmd_raw, -max_velocity_, max_velocity_);

        // Apply scaling from previous saturation feedback
        primary_cmd *= speed_scale_;
        const double omega_cmd_scaled = omega_cmd_raw * heading_scale_;

        LIBSTP_LOG_TRACE("DiagonalMotion scaled cmd: primary={:.3f}, omega={:.3f} (speed_scale={:.3f}, heading_scale={:.3f})",
                    primary_cmd, omega_cmd_scaled, speed_scale_, heading_scale_);

        // Rotate velocity from travel frame back to robot frame
        foundation::ChassisVelocity cmd{};
        cmd.vx = primary_cmd * cos_angle_;
        cmd.vy = primary_cmd * sin_angle_;
        cmd.wz = omega_cmd_scaled;

        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Record telemetry
        const auto sp = profiled_pid_.getSetpoint();
        telemetry_.push_back(DiagonalMotionTelemetry{
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
                "DiagonalMotion: Saturation detected (mask=0x{:X}, yaw_error={:.3f}) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
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
                        "DiagonalMotion: Recovery (after {} cycles) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                        unsaturated_cycles_,
                        prev_speed_scale, speed_scale_,
                        prev_heading_scale, heading_scale_);
                }
            }
        }
    }

    bool DiagonalMotion::isFinished() const
    {
        return finished_;
    }

    void DiagonalMotion::complete()
    {
        finished_ = true;
        drive().hardStop();
    }
}
