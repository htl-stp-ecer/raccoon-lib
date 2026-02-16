#include "motion/turn_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"

namespace
{
    constexpr double kDegToRad = std::numbers::pi / 180.0;
    constexpr double kSettlingVelocity = 0.05; // rad/s - must be nearly stopped to declare done
}

namespace libstp::motion
{
    static double computeAngularMaxVelocity(const UnifiedMotionPidConfig& pid_config,
                                              const TurnConfig& config)
    {
        double scale = std::clamp(config.speed_scale, 0.01, 1.0);
        return scale * pid_config.angular.max_velocity;
    }

    static ProfiledPIDController makeProfiledPID(const UnifiedMotionPidConfig& pid_config,
                                                  double max_velocity)
    {
        ProfiledPIDController::Config cfg;
        cfg.pid = pid_config.heading;
        cfg.velocity_ff = pid_config.velocity_ff;

        TrapezoidalProfile::Constraints constraints;
        constraints.max_velocity = max_velocity;
        constraints.max_acceleration = pid_config.angular.acceleration;
        constraints.max_deceleration = pid_config.angular.deceleration;

        return ProfiledPIDController(cfg, constraints);
    }

    TurnMotion::TurnMotion(MotionContext ctx, TurnConfig config)
        : Motion(ctx), cfg_(config)
        , max_velocity_(computeAngularMaxVelocity(ctx_.pid_config, config))
        , profiled_pid_(makeProfiledPID(ctx_.pid_config, max_velocity_))
    {
    }

    void TurnMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;

        prev_heading_ = 0.0;
        filtered_velocity_ = 0.0;

        odometry().reset();

        // Reset profiled PID at current position (0 after odometry reset)
        profiled_pid_.reset(0.0);
        profiled_pid_.setGoal(cfg_.target_angle_rad);

        LIBSTP_LOG_DEBUG("TurnMotion started: target={:.3f} rad ({:.1f} deg), "
                    "max_velocity={:.3f} rad/s (scale={:.2f}), "
                    "kP={:.3f}, kI={:.3f}, kD={:.3f}, vel_ff={:.3f}, kS={:.3f}",
                    cfg_.target_angle_rad, cfg_.target_angle_rad / kDegToRad,
                    max_velocity_, cfg_.speed_scale,
                    ctx_.pid_config.heading.kp, ctx_.pid_config.heading.ki,
                    ctx_.pid_config.heading.kd, ctx_.pid_config.velocity_ff, cfg_.kS);
    }

    void TurnMotion::update(double dt)
    {
        if (!started_) start();

        if (finished_)
        {
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            if (dt > 0.0) { [[maybe_unused]] const auto mc = drive().update(dt); }
            return;
        }

        if (dt <= 0.0) return;

        odometry().update(dt);

        const double current_heading = odometry().getHeading();

        // Filtered angular velocity for settling detection
        const double raw_velocity = (current_heading - prev_heading_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_heading_ = current_heading;

        // Heading error to goal for settling check
        const double error = odometry().getHeadingError(cfg_.target_angle_rad);

        // Settling: within tolerance AND nearly stopped
        if (std::abs(error) <= ctx_.pid_config.angle_tolerance_rad &&
            std::abs(filtered_velocity_) < kSettlingVelocity)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            [[maybe_unused]] const auto mc = drive().update(dt);
            LIBSTP_LOG_DEBUG("TurnMotion completed: heading={:.3f} rad, error={:.4f} rad ({:.2f} deg)",
                        current_heading, error, error / kDegToRad);
            return;
        }

        // Profiled PID: profile advances setpoint, PID tracks it
        double omega_cmd = profiled_pid_.calculate(current_heading, dt);

        // Static friction compensation (smooth linear ramp inside tolerance zone)
        if (cfg_.kS > 0.0)
        {
            const double smooth_zone = ctx_.pid_config.angle_tolerance_rad;
            const double kS_scale = std::clamp(error / smooth_zone, -1.0, 1.0);
            omega_cmd += cfg_.kS * kS_scale;
        }

        // Clamp to what the robot can actually achieve
        omega_cmd = std::clamp(omega_cmd, -max_velocity_, max_velocity_);

        drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, omega_cmd});
        [[maybe_unused]] const auto mc = drive().update(dt);

        const auto sp = profiled_pid_.getSetpoint();
        LIBSTP_LOG_DEBUG("TurnMotion: heading={:.3f}, error={:.4f}, setpoint={:.3f}, sp_vel={:.3f}, omega={:.3f}, filt_vel={:.3f}",
                    current_heading, error, sp.position, sp.velocity, omega_cmd, filtered_velocity_);
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
