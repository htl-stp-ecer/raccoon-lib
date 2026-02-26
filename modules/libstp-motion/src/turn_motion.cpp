#include "motion/turn_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "foundation/logging.hpp"
#include "odometry/angle_utils.hpp"

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

        cycle_++;
        odometry().update(dt);

        const double current_heading = odometry().getHeading();
        const double current_heading_deg = current_heading / kDegToRad;

        // Filtered angular velocity for settling detection
        const double raw_velocity = (current_heading - prev_heading_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_heading_ = current_heading;

        // Heading error to goal for settling check
        const double error = odometry().getHeadingError(cfg_.target_angle_rad);

        // Drive feedback: measured angular velocity from gyro (via drive's velocity PID)
        const auto drive_state = drive().estimateState();

        // Settling: within tolerance AND nearly stopped
        if (std::abs(error) <= ctx_.pid_config.angle_tolerance_rad &&
            std::abs(filtered_velocity_) < kSettlingVelocity)
        {
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            [[maybe_unused]] const auto mc = drive().update(dt);
            LIBSTP_LOG_INFO("TURN DONE [c={}] heading={:.1f}deg error={:.2f}deg filt_vel={:.4f}",
                        cycle_, current_heading_deg, error / kDegToRad, filtered_velocity_);
            return;
        }

        // Snapshot setpoint BEFORE calculate to see profile advance
        const auto sp_before = profiled_pid_.getSetpoint();

        // Profiled PID: profile advances setpoint, PID tracks it
        const double pid_raw = profiled_pid_.calculate(current_heading, dt);

        const auto sp = profiled_pid_.getSetpoint();
        const double tracking_error = sp.position - current_heading;
        const bool profile_done = profiled_pid_.profileComplete();

        double omega_cmd = pid_raw;

        // Static friction compensation (smooth linear ramp inside tolerance zone)
        if (cfg_.kS > 0.0)
        {
            const double smooth_zone = ctx_.pid_config.angle_tolerance_rad;
            const double kS_scale = std::clamp(error / smooth_zone, -1.0, 1.0);
            omega_cmd += cfg_.kS * kS_scale;
        }

        const double omega_unclamped = omega_cmd;

        // Clamp to what the robot can actually achieve
        omega_cmd = std::clamp(omega_cmd, -max_velocity_, max_velocity_);

        drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, omega_cmd});
        [[maybe_unused]] const auto mc = drive().update(dt);

        // --- Comprehensive diagnostic log ---
        // Line 1: Heading state
        LIBSTP_LOG_DEBUG(
            "TURN [c={} dt={:.4f}] hdg={:.1f}deg err={:.2f}deg target={:.1f}deg | "
            "sp={:.3f}->{:.3f}rad sp_vel={:.3f} trk_err={:.3f} prof_done={} | "
            "pid_raw={:.3f} omega={:.3f} (uncl={:.3f} max={:.3f}) | "
            "raw_vel={:.3f} filt_vel={:.3f} gyro_wz={:.3f}",
            cycle_, dt,
            current_heading_deg, error / kDegToRad, cfg_.target_angle_rad / kDegToRad,
            sp_before.position, sp.position, sp.velocity, tracking_error, profile_done,
            pid_raw, omega_cmd, omega_unclamped, max_velocity_,
            raw_velocity, filtered_velocity_, drive_state.wz);

        // Warn on anomalous conditions that likely indicate bugs
        if (std::abs(tracking_error) > 0.5) {
            LIBSTP_LOG_WARN(
                "TURN [c={}] LARGE TRACKING ERROR: {:.3f}rad ({:.1f}deg) - "
                "setpoint={:.3f} heading={:.3f} (profile running away from robot?)",
                cycle_, tracking_error, tracking_error / kDegToRad,
                sp.position, current_heading);
        }
        if (std::abs(raw_velocity) > 3.0 * max_velocity_) {
            LIBSTP_LOG_WARN(
                "TURN [c={}] HEADING JUMP: raw_vel={:.3f}rad/s (>{:.1f}x max) - "
                "heading={:.3f} prev={:.3f} (extractYaw discontinuity?)",
                cycle_, raw_velocity, 3.0, current_heading, current_heading - raw_velocity * dt);
        }
        if (std::abs(omega_cmd) >= max_velocity_ * 0.99 && std::abs(error) > 0.1) {
            LIBSTP_LOG_WARN(
                "TURN [c={}] SATURATED: omega={:.3f} (max={:.3f}) while error={:.1f}deg - "
                "gyro_wz={:.3f} (sign mismatch? drive not tracking?)",
                cycle_, omega_cmd, max_velocity_, error / kDegToRad, drive_state.wz);
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
