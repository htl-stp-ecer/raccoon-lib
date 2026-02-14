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
    static ProfiledPIDController makeProfiledPID(const UnifiedMotionPidConfig& pid_config,
                                                  const TurnConfig& turn_config)
    {
        ProfiledPIDController::Config cfg;
        cfg.kp = pid_config.heading_kp;
        cfg.ki = pid_config.heading_ki;
        cfg.kd = pid_config.heading_kd;
        cfg.velocity_ff = pid_config.velocity_ff;
        cfg.derivative_lpf_alpha = pid_config.derivative_lpf_alpha;
        cfg.integral_max = pid_config.integral_max;
        cfg.integral_deadband = pid_config.integral_deadband;

        TrapezoidalProfile::Constraints constraints;
        constraints.max_velocity = turn_config.max_angular_rate;
        constraints.max_acceleration = turn_config.max_angular_acceleration;
        constraints.max_deceleration = turn_config.max_angular_deceleration;

        return ProfiledPIDController(cfg, constraints);
    }

    TurnMotion::TurnMotion(MotionContext ctx, double angle_deg, double max_angular_rate_rad_per_sec)
        : TurnMotion(ctx, TurnConfig{
            .target_angle_rad = angle_deg * kDegToRad,
            .max_angular_rate = max_angular_rate_rad_per_sec
        })
    {
    }

    TurnMotion::TurnMotion(MotionContext ctx, foundation::Radians angle, foundation::RadiansPerSecond max_angular_rate)
        : TurnMotion(ctx, TurnConfig{
            .target_angle_rad = angle.value,
            .max_angular_rate = max_angular_rate.value
        })
    {
    }

    TurnMotion::TurnMotion(MotionContext ctx, TurnConfig config)
        : Motion(ctx), cfg_(config)
        , profiled_pid_(makeProfiledPID(ctx_.pid_config, config))
    {
        if (cfg_.max_angular_rate <= 0.0) cfg_.max_angular_rate = 0.5;
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
                    "max_rate={:.3f} rad/s, max_accel={:.3f} rad/s², max_decel={:.3f} rad/s², "
                    "kP={:.3f}, kI={:.3f}, kD={:.3f}, vel_ff={:.3f}, kS={:.3f}",
                    cfg_.target_angle_rad, cfg_.target_angle_rad / kDegToRad,
                    cfg_.max_angular_rate, cfg_.max_angular_acceleration,
                    cfg_.max_angular_deceleration,
                    ctx_.pid_config.heading_kp, ctx_.pid_config.heading_ki,
                    ctx_.pid_config.heading_kd, ctx_.pid_config.velocity_ff, cfg_.kS);
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
        omega_cmd = std::clamp(omega_cmd, -cfg_.max_angular_rate, cfg_.max_angular_rate);

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
