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
        // Validate configuration parameters
        if (cfg_.max_speed_mps <= 0.0) cfg_.max_speed_mps = 0.3;

        // Create PID controllers using factory (eliminates boilerplate)
        lateral_pid_ = createPidController(ctx_.pid_config, PidType::Lateral);
        heading_pid_ = createPidController(ctx_.pid_config, PidType::Heading);
    }

    void StrafeMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        elapsed_time_ = 0.0;
        unsaturated_cycles_ = 0;

        // Reset odometry to establish new origin for this motion
        odometry().reset();

        // Reset PID controllers
        lateral_pid_->reset();
        heading_pid_->reset();

        // Store initial heading (should be ~0 after reset) and target distance
        initial_heading_rad_ = odometry().getHeading();
        target_distance_m_ = cfg_.target_distance_m;

        // Create trapezoidal profile for smooth motion
        TrapezoidalProfile::State initial{0.0, 0.0};  // Start at position=0, velocity=0
        TrapezoidalProfile::Constraints constraints{
            .max_velocity = cfg_.max_speed_mps,
            .max_acceleration = cfg_.max_acceleration_mps2
        };
        profile_ = std::make_unique<TrapezoidalProfile>(initial, target_distance_m_, constraints);

        LIBSTP_LOG_TRACE("StrafeMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s, profile_time = {:.3f} s",
                    cfg_.target_distance_m, cfg_.max_speed_mps, profile_->getTotalTime());
    }

    void StrafeMotion::update(double dt)
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
            LIBSTP_LOG_WARN("StrafeMotion::update called with invalid dt={:.6f}s (must be > 0)", dt);
            return;
        }

        // Update elapsed time
        elapsed_time_ += dt;

        // Update odometry first
        odometry().update(dt);

        // Get setpoint from trapezoidal profile
        const auto setpoint = profile_->getSetpoint(elapsed_time_);

        // Get distance from origin - lateral is perpendicular to initial forward direction
        // Note: positive lateral (from odometry) = moved right, negative = moved left
        const auto distance_info = odometry().getDistanceFromOrigin();
        const double current_lateral_m = distance_info.lateral;  // Use as-is: positive=right, negative=left
        const double current_heading = odometry().getHeading();

        // Compute lateral distance error against the ramped setpoint (not the final target!)
        const double lateral_error = setpoint.position - current_lateral_m;
        const double error_to_final_target = target_distance_m_ - current_lateral_m;

        // Compute heading error (maintain initial heading)
        const double heading_error = odometry().getHeadingError(initial_heading_rad_);

        LIBSTP_LOG_TRACE("StrafeMotion update: current_lateral = {:.3f} m, setpoint = {:.3f} m, target = {:.3f} m, lateral_error = {:.3f} m, heading_error = {:.3f} rad",
                    current_lateral_m, setpoint.position, target_distance_m_, lateral_error, heading_error);

        // Check if we've reached the final target distance
        const double error_abs = std::abs(error_to_final_target);
        if (error_abs <= ctx_.pid_config.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            LIBSTP_LOG_TRACE("StrafeMotion completed: final error = {:.3f} m", lateral_error);
            return;
        }

        // Compute lateral velocity using PID control
        double vy_cmd = lateral_pid_->update(lateral_error, dt);

        // Apply minimum speed to prevent stalling
        if (error_abs > ctx_.pid_config.distance_tolerance_m && std::abs(vy_cmd) < ctx_.pid_config.min_speed_mps)
        {
            const double direction = (lateral_error >= 0.0) ? 1.0 : -1.0;
            vy_cmd = direction * ctx_.pid_config.min_speed_mps;
            LIBSTP_LOG_TRACE("StrafeMotion: Applying minimum speed: vy = {:.3f} m/s", vy_cmd);
        }

        // Apply scaling from previous saturation feedback
        const double vy_cmd_scaled = vy_cmd * speed_scale_;

        // Compute heading correction to maintain straight strafe
        const double omega_correction = heading_pid_->update(heading_error, dt);

        LIBSTP_LOG_TRACE("StrafeMotion: vy_cmd = {:.3f} m/s, scaled = {:.3f} m/s (scale={:.3f}), omega_correction = {:.3f} rad/s",
                    vy_cmd, vy_cmd_scaled, speed_scale_, omega_correction);

        // Send command: no forward motion, lateral strafe, with heading correction
        foundation::ChassisVelocity cmd{0.0, vy_cmd_scaled, omega_correction};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback with hysteresis to prevent oscillation
        if (motor_cmd.saturated_any)
        {
            // Reset hysteresis counter on saturation
            unsaturated_cycles_ = 0;

            const double prev_scale = speed_scale_;
            speed_scale_ = std::max(
                ctx_.pid_config.saturation_min_scale,
                speed_scale_ * ctx_.pid_config.saturation_derating_factor);

            LIBSTP_LOG_TRACE("StrafeMotion: Saturation detected (mask=0x{:X}) -> speed_scale {:.3f}->{:.3f}",
                        motor_cmd.saturation_mask, prev_scale, speed_scale_);
        }
        else
        {
            // Hysteresis: only recover after sustained period without saturation
            ++unsaturated_cycles_;

            const bool can_recover = unsaturated_cycles_ >= ctx_.pid_config.saturation_hold_cycles;
            const bool needs_recovery = speed_scale_ < ctx_.pid_config.saturation_recovery_threshold;

            if (can_recover && needs_recovery)
            {
                const double prev_scale = speed_scale_;
                speed_scale_ = std::min(1.0, speed_scale_ + ctx_.pid_config.saturation_recovery_rate);

                if (prev_scale != speed_scale_)
                {
                    LIBSTP_LOG_TRACE("StrafeMotion: Recovery (after {} cycles) -> speed_scale {:.3f}->{:.3f}",
                                unsaturated_cycles_, prev_scale, speed_scale_);
                }
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
