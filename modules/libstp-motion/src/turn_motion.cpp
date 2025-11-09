#include "motion/turn_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "spdlog/spdlog.h"

namespace
{
    constexpr double kDegToRad = std::numbers::pi / 180.0;
    constexpr double kRadToDeg = 180.0 / std::numbers::pi;
}

namespace libstp::motion
{
    TurnMotion::TurnMotion(MotionContext ctx, double angle_deg, double max_angular_speed_rps)
        : TurnMotion(ctx, TurnMotionConfig{
            .angle_deg = angle_deg,
            .max_angular_speed_rps = max_angular_speed_rps
        })
    {
    }

    TurnMotion::TurnMotion(MotionContext ctx, TurnMotionConfig config)
        : Motion(ctx), cfg_(config)
    {
        cfg_.max_angular_speed_rps = std::abs(cfg_.max_angular_speed_rps);
        if (cfg_.max_angular_speed_rps <= 0.0) cfg_.max_angular_speed_rps = 0.5;
        if (cfg_.angle_tolerance_deg <= 0.0) cfg_.angle_tolerance_deg = 1.0;
        if (cfg_.angle_kp <= 0.0) cfg_.angle_kp = 1.0;
        cfg_.saturation_derating_factor = std::clamp(cfg_.saturation_derating_factor, 0.1, 0.99);
        cfg_.saturation_min_speed_scale = std::clamp(cfg_.saturation_min_speed_scale, 0.01, 1.0);
        cfg_.saturation_recovery_rate = std::clamp(cfg_.saturation_recovery_rate, 0.0, 0.5);
    }

    void TurnMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        angular_scale_ = 1.0;
        cumulative_angle_rad_ = 0.0;

        const auto pose = odometry().getPose();
        reference_orientation_ = pose.orientation.normalized();
        previous_angle_rad_ = 0.0;  // Start at 0 relative rotation

        // Store the target angle to turn
        target_angle_rad_ = cfg_.angle_deg * kDegToRad;

        const double reference_yaw = computeYaw(reference_orientation_);

        SPDLOG_INFO("TurnMotion started: target_angle = {:.3f} deg ({:.3f} rad), max_angular_speed = {:.3f} rad/s, reference_yaw = {:.3f} rad",
                    cfg_.angle_deg, target_angle_rad_, cfg_.max_angular_speed_rps, reference_yaw);
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

        // Update odometry first
        odometry().update(dt);

        const auto pose = odometry().getPose();
        const double current_yaw = computeYaw(pose.orientation);

        // Compute rotation from reference orientation to current orientation
        Eigen::Quaternionf current_orientation = pose.orientation.normalized();

        // Ensure quaternions are in the same hemisphere (q and -q represent the same rotation)
        if (reference_orientation_.dot(current_orientation) < 0.0f)
        {
            current_orientation.coeffs() *= -1.0f;
        }

        // Compute the relative rotation: how much we've rotated from the start
        Eigen::Quaternionf delta_q = current_orientation * reference_orientation_.conjugate();
        delta_q.normalize();

        // Extract the signed rotation angle around the Z-axis (yaw)
        // For a quaternion q = [w, x, y, z], the rotation angle around Z is:
        // angle = 2 * atan2(z, w)
        // This gives us the signed angle in the range [-π, π]
        double current_angle_rad = 2.0 * std::atan2(static_cast<double>(delta_q.z()),
                                                     static_cast<double>(delta_q.w()));

        // Track cumulative rotation to handle angles beyond ±180°
        // Detect wrapping by checking if the angle jumped by more than π
        double delta_angle = current_angle_rad - previous_angle_rad_;
        const double pi = std::numbers::pi;

        if (delta_angle > pi)
        {
            // Wrapped from +π to -π (turned counter-clockwise past 180°)
            delta_angle -= 2.0 * pi;
        }
        else if (delta_angle < -pi)
        {
            // Wrapped from -π to +π (turned clockwise past -180°)
            delta_angle += 2.0 * pi;
        }

        cumulative_angle_rad_ += delta_angle;
        previous_angle_rad_ = current_angle_rad;

        SPDLOG_INFO("TurnMotion update: current_yaw = {:.3f} rad ({:.3f} deg), cumulative_angle = {:.3f} rad ({:.3f} deg), delta_q = ({:.3f}, {:.3f}, {:.3f}, {:.3f})",
                    current_yaw, current_yaw * kRadToDeg, cumulative_angle_rad_, cumulative_angle_rad_ * kRadToDeg,
                    delta_q.w(), delta_q.x(), delta_q.y(), delta_q.z());

        // Compute remaining angle to turn using cumulative tracking
        double remaining_angle_rad = target_angle_rad_ - cumulative_angle_rad_;
        const double remaining_angle_deg = remaining_angle_rad * kRadToDeg;

        SPDLOG_INFO("TurnMotion remaining_angle = {:.3f} deg ({:.3f} rad)",
                    remaining_angle_deg, remaining_angle_rad);

        // Check if we've reached the target
        if (std::abs(remaining_angle_deg) <= cfg_.angle_tolerance_deg)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            SPDLOG_INFO("TurnMotion completed: final_yaw = {:.3f} rad ({:.3f} deg), total_turned = {:.3f} deg",
                        current_yaw, current_yaw * kRadToDeg, cumulative_angle_rad_ * kRadToDeg);
            return;
        }

        // Compute angular velocity command using proportional control
        double omega_cmd = std::clamp(cfg_.angle_kp * remaining_angle_rad,
                                      -cfg_.max_angular_speed_rps,
                                      cfg_.max_angular_speed_rps);

        // Ensure minimum angular velocity to avoid getting stuck
        if (std::abs(omega_cmd) < 1e-4)
        {
            const double direction = (remaining_angle_rad >= 0.0) ? 1.0 : -1.0;
            omega_cmd = direction * std::min(cfg_.max_angular_speed_rps, 0.1);
        }

        // Apply scaling from previous saturation feedback
        omega_cmd *= angular_scale_;

        SPDLOG_INFO("TurnMotion omega_cmd = {:.3f} rad/s (angular_scale={:.3f})",
                    omega_cmd, angular_scale_);

        // Send command (no translation, only rotation)
        foundation::ChassisVel cmd{0.0, 0.0, omega_cmd};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback
        if (motor_cmd.saturated_any)
        {
            const double prev_angular_scale = angular_scale_;
            angular_scale_ = std::max(
                cfg_.saturation_min_speed_scale,
                angular_scale_ * cfg_.saturation_derating_factor);

            SPDLOG_INFO("TurnMotion: Saturation detected (mask=0x{:X}) -> angular_scale {:.3f}->{:.3f}",
                        motor_cmd.saturation_mask, prev_angular_scale, angular_scale_);
        }
        else
        {
            const double prev_angular_scale = angular_scale_;
            angular_scale_ = std::min(1.0, angular_scale_ + cfg_.saturation_recovery_rate);

            if (prev_angular_scale != angular_scale_)
            {
                SPDLOG_INFO("TurnMotion: Recovery -> angular_scale {:.3f}->{:.3f}",
                            prev_angular_scale, angular_scale_);
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
    }

    double TurnMotion::computeYaw(const Eigen::Quaternionf& orientation) const
    {
        const Eigen::Quaternionf q = orientation.normalized();
        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double TurnMotion::wrapAngle(double angle)
    {
        const double two_pi = 2.0 * std::numbers::pi;
        double wrapped = std::fmod(angle + std::numbers::pi, two_pi);
        if (wrapped < 0.0) wrapped += two_pi;
        return wrapped - std::numbers::pi;
    }
}
