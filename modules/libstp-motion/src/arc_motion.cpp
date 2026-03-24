#include "motion/arc_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace libstp::motion
{
    static double computeArcMaxAngularVelocity(const UnifiedMotionPidConfig& pid_config,
                                                const ArcMotionConfig& config)
    {
        double scale = std::clamp(config.speed_scale, 0.01, 1.0);

        // Angular velocity is limited by both angular and linear axis constraints.
        // Since vx = |omega| * radius, we need: |omega| * radius <= linear.max_velocity.
        double angular_limit = pid_config.angular.max_velocity;
        double linear_limit = (config.radius_m > 1e-6)
            ? pid_config.linear.max_velocity / config.radius_m
            : angular_limit;

        return scale * std::min(angular_limit, linear_limit);
    }

    static ProfiledPIDController makeArcProfiledPID(const UnifiedMotionPidConfig& pid_config,
                                                     const ArcMotionConfig& config,
                                                     double max_angular_velocity)
    {
        ProfiledPIDController::Config cfg;
        cfg.pid = pid_config.heading;
        cfg.velocity_ff = pid_config.velocity_ff;

        // Scale acceleration limits similarly: angular accel must not produce
        // linear acceleration exceeding the linear axis limits.
        double angular_accel = pid_config.angular.acceleration;
        double angular_decel = pid_config.angular.deceleration;
        if (config.radius_m > 1e-6)
        {
            angular_accel = std::min(angular_accel, pid_config.linear.acceleration / config.radius_m);
            angular_decel = std::min(angular_decel, pid_config.linear.deceleration / config.radius_m);
        }

        TrapezoidalProfile::Constraints constraints;
        constraints.max_velocity = max_angular_velocity;
        constraints.max_acceleration = angular_accel;
        constraints.max_deceleration = angular_decel;

        return ProfiledPIDController(cfg, constraints);
    }

    ArcMotion::ArcMotion(MotionContext ctx, ArcMotionConfig config)
        : Motion(ctx), cfg_(config)
        , max_angular_velocity_(computeArcMaxAngularVelocity(ctx_.pid_config, config))
        , profiled_pid_(makeArcProfiledPID(ctx_.pid_config, config, max_angular_velocity_))
    {
    }

    void ArcMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        cycle_ = 0;

        prev_heading_ = 0.0;
        filtered_velocity_ = 0.0;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        odometry().reset();

        profiled_pid_.reset(0.0);
        profiled_pid_.setGoal(cfg_.arc_angle_rad);

        LIBSTP_LOG_DEBUG("ArcMotion started: radius={:.3f} m, arc_angle={:.3f} rad ({:.1f} deg), "
                    "max_omega={:.3f} rad/s, max_v={:.3f} m/s (scale={:.2f}, lateral={})",
                    cfg_.radius_m,
                    cfg_.arc_angle_rad, cfg_.arc_angle_rad * 180.0 / std::numbers::pi,
                    max_angular_velocity_, max_angular_velocity_ * cfg_.radius_m,
                    cfg_.speed_scale, cfg_.lateral);
    }

    void ArcMotion::update(double dt)
    {
        if (!started_) start();

        if (finished_)
        {
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            if (dt > 0.0) { [[maybe_unused]] const auto mc = drive().update(dt); }
            return;
        }

        if (dt <= 0.0) return;

        cycle_++;
        elapsed_time_ += dt;

        odometry().update(dt);

        const double current_heading = odometry().getHeading();
        const double heading_error = odometry().getHeadingError(cfg_.arc_angle_rad);

        // Filtered angular velocity for settling detection
        const double raw_velocity = (current_heading - prev_heading_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_heading_ = current_heading;

        // Settling: within angle tolerance AND nearly stopped
        if (std::abs(heading_error) <= ctx_.pid_config.angle_tolerance_rad &&
            std::abs(filtered_velocity_) < kSettlingVelocity)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            [[maybe_unused]] const auto mc = drive().update(dt);
            LIBSTP_LOG_INFO("ARC DONE [c={}] heading={:.1f}deg error={:.2f}deg filt_vel={:.4f}",
                        cycle_,
                        current_heading * 180.0 / std::numbers::pi,
                        heading_error * 180.0 / std::numbers::pi,
                        filtered_velocity_);
            return;
        }

        // Profiled PID on heading: profile advances setpoint, PID tracks it
        const double omega_cmd_raw = profiled_pid_.calculate(current_heading, dt);
        const double omega_cmd = std::clamp(omega_cmd_raw, -max_angular_velocity_, max_angular_velocity_);

        // Derive linear velocity from angular velocity: v = |omega| * radius
        // For lateral mode (strafe arc), velocity goes to vy; otherwise to vx.
        const double linear_cmd = std::abs(omega_cmd) * cfg_.radius_m;
        const double vx_cmd = cfg_.lateral ? 0.0 : linear_cmd;
        // Strafe sign: positive arc_angle (CCW/left turn) => strafe right (+vy)
        //              negative arc_angle (CW/right turn) => strafe left (-vy)
        const double vy_sign = (cfg_.arc_angle_rad >= 0.0) ? 1.0 : -1.0;
        const double vy_cmd = cfg_.lateral ? vy_sign * linear_cmd : 0.0;

        drive().setVelocity(foundation::ChassisVelocity{vx_cmd, vy_cmd, omega_cmd});
        [[maybe_unused]] const auto mc = drive().update(dt);

        // Record telemetry
        const auto sp = profiled_pid_.getSetpoint();
        const double arc_position = std::abs(current_heading) * cfg_.radius_m;
        const double arc_target = std::abs(cfg_.arc_angle_rad) * cfg_.radius_m;

        telemetry_.push_back(ArcMotionTelemetry{
            .time_s = elapsed_time_,
            .dt = dt,
            .target_angle_rad = cfg_.arc_angle_rad,
            .heading_rad = current_heading,
            .heading_error_rad = heading_error,
            .arc_position_m = arc_position,
            .arc_target_m = arc_target,
            .filtered_velocity_radps = filtered_velocity_,
            .cmd_vx_mps = vx_cmd,
            .cmd_vy_mps = vy_cmd,
            .cmd_wz_radps = omega_cmd,
            .pid_raw = omega_cmd_raw,
            .setpoint_position_rad = sp.position,
            .setpoint_velocity_radps = sp.velocity,
            .saturated = mc.saturated_any,
        });

        LIBSTP_LOG_DEBUG(
            "ARC [c={} dt={:.4f}] hdg={:.1f}deg err={:.2f}deg | "
            "sp={:.3f}rad sp_vel={:.3f} | "
            "omega={:.3f} vx={:.3f} vy={:.3f} | filt_vel={:.3f}",
            cycle_, dt,
            current_heading * 180.0 / std::numbers::pi,
            heading_error * 180.0 / std::numbers::pi,
            sp.position, sp.velocity,
            omega_cmd, vx_cmd, vy_cmd, filtered_velocity_);
    }

    bool ArcMotion::isFinished() const
    {
        return finished_;
    }

    void ArcMotion::complete()
    {
        finished_ = true;
        drive().hardStop();
    }
}
