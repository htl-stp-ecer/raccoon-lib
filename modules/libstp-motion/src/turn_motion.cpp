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
    {
        if (cfg_.max_angular_rate <= 0.0) cfg_.max_angular_rate = 0.5;

        kP_ = ctx_.pid_config.heading_kp;
        kI_ = ctx_.pid_config.heading_ki;
        kD_ = ctx_.pid_config.heading_kd;
    }

    void TurnMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;

        prev_error_ = 0.0;
        total_error_ = 0.0;
        filtered_derivative_ = 0.0;
        last_error_sign_ = 0;
        prev_heading_ = 0.0;
        filtered_velocity_ = 0.0;

        odometry().reset();

        LIBSTP_LOG_DEBUG("TurnMotion started: target={:.3f} rad ({:.1f} deg), "
                    "max_rate={:.3f} rad/s, kP={:.3f}, kI={:.3f}, kD={:.3f}, kS={:.3f}",
                    cfg_.target_angle_rad, cfg_.target_angle_rad / kDegToRad,
                    cfg_.max_angular_rate, kP_, kI_, kD_, cfg_.kS);
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

        // Heading error to goal (shortest path, handles wrapping)
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

        // --- PID on heading error ---
        // P * error saturated by max_angular_rate naturally creates a
        // trapezoidal-like velocity profile: full speed while far, proportional
        // deceleration as we approach.  D damps momentum to prevent overshoot.

        // Filtered derivative (EMA low-pass to reduce sensor noise)
        const double raw_derivative = (error - prev_error_) / dt;
        const double alpha = ctx_.pid_config.derivative_lpf_alpha;
        filtered_derivative_ = alpha * raw_derivative + (1.0 - alpha) * filtered_derivative_;

        // Integral with deadband and zero-crossing decay
        const int error_sign = (error > 0.0) ? 1 : ((error < 0.0) ? -1 : 0);
        if (last_error_sign_ != 0 && error_sign != 0 && error_sign != last_error_sign_)
        {
            total_error_ *= 0.5;  // Decay accumulated integral on sign change
        }
        if (error_sign != 0) last_error_sign_ = error_sign;

        if (std::abs(error) > ctx_.pid_config.integral_deadband)
        {
            total_error_ += error * dt;
        }
        const double integral_max = ctx_.pid_config.integral_max;
        total_error_ = std::clamp(total_error_, -integral_max, integral_max);

        prev_error_ = error;

        double omega_cmd = kP_ * error;
        omega_cmd += kI_ * total_error_;
        omega_cmd += kD_ * filtered_derivative_;

        // Smooth static friction compensation (linear ramp inside tolerance zone)
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

        LIBSTP_LOG_DEBUG("TurnMotion: heading={:.3f}, error={:.4f}, omega={:.3f}, filt_vel={:.3f}",
                    current_heading, error, omega_cmd, filtered_velocity_);
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
