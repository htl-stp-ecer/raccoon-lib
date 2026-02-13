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

        distance_pid_ = createPidController(ctx_.pid_config, PidType::Distance);
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
        elapsed_time_ = 0.0;
        unsaturated_cycles_ = 0;

        odometry().reset();

        distance_pid_->reset();
        heading_pid_->reset();
        cross_track_pid_->reset();

        initial_heading_rad_ = odometry().getHeading();

        TrapezoidalProfile::State initial{0.0, 0.0};
        TrapezoidalProfile::Constraints constraints{
            .max_velocity = cfg_.max_speed_mps,
            .max_acceleration = cfg_.max_acceleration_mps2
        };
        profile_ = std::make_unique<TrapezoidalProfile>(initial, cfg_.distance_m, constraints);

        LIBSTP_LOG_TRACE("LinearMotion started: axis={}, target={:.3f} m, max_speed={:.3f} m/s, profile_time={:.3f} s",
                    (cfg_.axis == LinearAxis::Forward ? "Forward" : "Lateral"),
                    cfg_.distance_m, cfg_.max_speed_mps, profile_->getTotalTime());
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

        const auto setpoint = profile_->getSetpoint(elapsed_time_);
        const auto distance_info = odometry().getDistanceFromOrigin();
        const double current_heading = odometry().getHeading();
        const double yaw_error = odometry().getHeadingError(initial_heading_rad_);

        // Extract axis-dependent positions
        const bool is_forward = (cfg_.axis == LinearAxis::Forward);
        const double primary_position = is_forward ? distance_info.forward : distance_info.lateral;
        const double cross_track_position = is_forward ? distance_info.lateral : distance_info.forward;

        const double distance_error = setpoint.position - primary_position;
        const double error_to_final_target = cfg_.distance_m - primary_position;

        LIBSTP_LOG_TRACE("LinearMotion update: primary={:.3f} m, setpoint={:.3f} m, target={:.3f} m, cross_track={:.3f} m, heading={:.3f} rad, yaw_error={:.3f} rad",
                    primary_position, setpoint.position, cfg_.distance_m, cross_track_position, current_heading, yaw_error);

        // Check if we've reached the final target distance
        const double remaining = error_to_final_target;
        const double remaining_abs = std::abs(remaining);

        if (remaining_abs <= ctx_.pid_config.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            return;
        }

        // Cross-track correction
        const double cross_track_error = cross_track_position;
        const bool supports_lateral = drive().getKinematics().supportsLateralMotion();

        double cross_cmd = 0.0;
        double omega_cmd = 0.0;

        if (supports_lateral)
        {
            // MECANUM: direct cross-track correction
            // Cross-track error sign: positive cross_track_position means drifted in positive direction
            // We want negative correction to push back
            cross_cmd = -cross_track_pid_->update(cross_track_error, dt);
            cross_cmd = std::clamp(cross_cmd, -cfg_.max_speed_mps * 0.5, cfg_.max_speed_mps * 0.5);
            omega_cmd = heading_pid_->update(yaw_error, dt);
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

                omega_cmd = heading_pid_->update(yaw_to_target, dt);

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
                omega_cmd = heading_pid_->update(biased_yaw_error, dt);
            }
        }

        // Primary axis velocity control
        double primary_cmd = distance_pid_->update(distance_error, dt);

        if (std::abs(primary_cmd) < 1e-4)
        {
            const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
            primary_cmd = direction * std::min(cfg_.max_speed_mps, ctx_.pid_config.min_speed_mps);
        }

        // Reduce primary speed during reorientation (differential only)
        if (reorienting_)
        {
            primary_cmd *= ctx_.pid_config.reorientation_speed_factor;
        }

        // Apply scaling from previous saturation feedback
        primary_cmd *= speed_scale_;
        cross_cmd *= speed_scale_;
        const double omega_cmd_scaled = omega_cmd * heading_scale_;

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
    }
}
