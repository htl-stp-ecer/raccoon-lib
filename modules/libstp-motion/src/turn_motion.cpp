#include "motion/turn_motion.hpp"
#include "motion/motion_pid.hpp"

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

        pid_ = createPidController(ctx_.pid_config, PidType::Heading);
    }

    void TurnMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        prev_heading_ = 0.0;

        odometry().reset();
        pid_->reset();

        target_heading_rad_ = cfg_.target_angle_rad;

        LIBSTP_LOG_DEBUG("TurnMotion started: target={:.3f} rad ({:.1f} deg), max_rate={:.3f} rad/s",
                    cfg_.target_angle_rad, cfg_.target_angle_rad / kDegToRad, cfg_.max_angular_rate);
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
        const double error = odometry().getHeadingError(target_heading_rad_);
        const double error_abs = std::abs(error);

        // Measure actual angular velocity for settling check
        const double angular_velocity = (current_heading - prev_heading_) / dt;
        prev_heading_ = current_heading;

        // Normalize error by target angle so PID gains are angle-independent.
        const double target_abs = std::abs(target_heading_rad_);
        const double normalized_error = (target_abs > 1e-6) ? (error / target_abs) : 0.0;

        // Check completion: position within tolerance AND nearly stopped
        if (error_abs <= ctx_.pid_config.angle_tolerance_rad &&
            std::abs(angular_velocity) < kSettlingVelocity)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            [[maybe_unused]] const auto mc = drive().update(dt);
            LIBSTP_LOG_DEBUG("TurnMotion completed: heading={:.3f} rad, error={:.4f} rad ({:.2f} deg)",
                        current_heading, error, error / kDegToRad);
            return;
        }

        // PID on normalized error, scale to angular velocity
        const double omega_cmd = pid_->update(normalized_error, dt) * cfg_.max_angular_rate;

        drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, omega_cmd});
        [[maybe_unused]] const auto mc = drive().update(dt);

        LIBSTP_LOG_DEBUG("TurnMotion: heading={:.3f}, error={:.3f}, norm_err={:.3f}, omega={:.3f}, ang_vel={:.3f}",
                    current_heading, error, normalized_error, omega_cmd, angular_velocity);
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
