#include "motion/turn_motion.hpp"
#include "motion/motion_pid.hpp"

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
        // Validate configuration parameters
        if (cfg_.max_angular_rate <= 0.0) cfg_.max_angular_rate = 0.5;

        const auto& pid_cfg = ctx_.pid_config;

        // Create PID controller using unified config
        MotionPidController::Config angle_pid_cfg;
        angle_pid_cfg.kp = pid_cfg.heading_kp;  // Use heading PID gains for turn motion
        angle_pid_cfg.ki = pid_cfg.heading_ki;
        angle_pid_cfg.kd = pid_cfg.heading_kd;
        angle_pid_cfg.output_min = pid_cfg.output_min;
        angle_pid_cfg.output_max = pid_cfg.output_max;
        angle_pid_cfg.integral_max = pid_cfg.integral_max;
        angle_pid_cfg.integral_deadband = pid_cfg.integral_deadband;
        angle_pid_cfg.derivative_lpf_alpha = pid_cfg.derivative_lpf_alpha;

        angle_pid_ = std::make_unique<MotionPidController>(angle_pid_cfg);
    }

    void TurnMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        angular_scale_ = 1.0;
        elapsed_time_ = 0.0;

        // Reset odometry to establish new origin for this motion
        // This zeros the heading, so our initial heading is 0.0
        odometry().reset();

        // Reset PID controller state
        angle_pid_->reset();

        // Target heading is simply the desired turn angle (since we reset to 0)
        target_heading_rad_ = cfg_.target_angle_rad;

        // Create trapezoidal profile for smooth motion
        TrapezoidalProfile::State initial{0.0, 0.0};  // Start at heading=0, velocity=0
        TrapezoidalProfile::Constraints constraints{
            .max_velocity = cfg_.max_angular_rate,
            .max_acceleration = ctx_.pid_config.max_angular_acceleration
        };
        profile_ = std::make_unique<TrapezoidalProfile>(initial, target_heading_rad_, constraints);

        LIBSTP_LOG_TRACE("TurnMotion started: target_angle = {:.3f} rad ({:.1f} deg), max_angular_rate = {:.3f} rad/s, profile_time = {:.3f} s",
                    cfg_.target_angle_rad, cfg_.target_angle_rad / kDegToRad, cfg_.max_angular_rate, profile_->getTotalTime());
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

        // Update elapsed time
        elapsed_time_ += dt;

        // Update odometry first
        odometry().update(dt);

        // Get setpoint from trapezoidal profile
        const auto setpoint = profile_->getSetpoint(elapsed_time_);
        const double current_heading = odometry().getHeading();

        // Compute error against the ramped setpoint (not the final target!)
        const double heading_error = odometry().getHeadingError(setpoint.position);
        const double error_to_final_target = odometry().getHeadingError(target_heading_rad_);

        LIBSTP_LOG_TRACE("TurnMotion update: current = {:.3f} rad ({:.1f} deg), setpoint = {:.3f} rad ({:.1f} deg), target = {:.3f} rad ({:.1f} deg), error = {:.3f} rad ({:.1f} deg)",
                    current_heading, current_heading / kDegToRad,
                    setpoint.position, setpoint.position / kDegToRad,
                    target_heading_rad_, target_heading_rad_ / kDegToRad,
                    heading_error, heading_error / kDegToRad);

        // Check if we've reached the final target angle
        const double error_abs = std::abs(error_to_final_target);
        if (error_abs <= ctx_.pid_config.angle_tolerance_rad)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            LIBSTP_LOG_DEBUG("TurnMotion completed: final error = {:.3f} rad ({:.1f} deg)", heading_error, heading_error / kDegToRad);
            return;
        }

        // Compute angular velocity using PID control
        // Positive error -> need to turn CCW (positive omega)
        // Negative error -> need to turn CW (negative omega)
        double omega_cmd = angle_pid_->update(heading_error, dt);

        // Apply minimum angular rate to prevent stalling
        // Only apply minimum if error is still significant
        if (error_abs > ctx_.pid_config.angle_tolerance_rad && std::abs(omega_cmd) < ctx_.pid_config.min_angular_rate)
        {
            const double direction = (heading_error >= 0.0) ? 1.0 : -1.0;
            omega_cmd = direction * ctx_.pid_config.min_angular_rate;
            LIBSTP_LOG_TRACE("TurnMotion: Applying minimum angular rate: omega = {:.3f} rad/s", omega_cmd);
        }

        // Apply scaling from previous saturation feedback
        const double omega_cmd_scaled = omega_cmd * angular_scale_;
        LIBSTP_LOG_TRACE("TurnMotion: omega_cmd = {:.3f} rad/s, scaled = {:.3f} rad/s (scale={:.3f})",
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
                ctx_.pid_config.saturation_min_scale,
                angular_scale_ * ctx_.pid_config.saturation_derating_factor);

            LIBSTP_LOG_TRACE("TurnMotion: Saturation detected (mask=0x{:X}) -> angular_scale {:.3f}->{:.3f}",
                        motor_cmd.saturation_mask, prev_scale, angular_scale_);
        }
        else
        {
            // Recover scaling when not saturated
            const double prev_scale = angular_scale_;
            angular_scale_ = std::min(1.0, angular_scale_ + ctx_.pid_config.saturation_recovery_rate);

            if (prev_scale != angular_scale_)
            {
                LIBSTP_LOG_TRACE("TurnMotion: Recovery -> angular_scale {:.3f}->{:.3f}",
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
        drive().hardStop();
    }
}
