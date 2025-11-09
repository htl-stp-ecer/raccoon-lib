#include "motion/drive_straight_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"
#include "spdlog/spdlog.h"

namespace
{
    constexpr double kCmToM = 0.01;
}

namespace libstp::motion
{
    DriveStraightMotion::DriveStraightMotion(MotionContext ctx, double distance_cm, double max_speed_mps)
        : DriveStraightMotion(ctx, DriveStraightConfig{
            .distance_m = distance_cm * kCmToM,
            .max_speed_mps = max_speed_mps
        })
    {
    }

    DriveStraightMotion::DriveStraightMotion(MotionContext ctx, DriveStraightConfig config)
        : Motion(ctx), cfg_(config)
    {
        cfg_.max_speed_mps = std::abs(cfg_.max_speed_mps);
        if (cfg_.max_speed_mps <= 0.0) cfg_.max_speed_mps = 0.05;
        if (cfg_.distance_tolerance_m <= 0.0) cfg_.distance_tolerance_m = 0.005;
        if (cfg_.distance_kp <= 0.0) cfg_.distance_kp = 1.0;
        if (cfg_.heading_kp < 0.0) cfg_.heading_kp = 0.0;
        if (cfg_.max_heading_rate < 0.0) cfg_.max_heading_rate = 0.0;
    }

    void DriveStraightMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        distance_travelled_m_ = 0.0;
        speed_scale_ = 1.0;

        const auto pose = odometry().getPose();
        reference_orientation_ = pose.orientation.normalized();
        const double reference_yaw = computeYaw(reference_orientation_);
        SPDLOG_INFO("DriveStraightMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s, reference_yaw = {:.3f} rad",
                    cfg_.distance_m, cfg_.max_speed_mps, reference_yaw);
    }

    void DriveStraightMotion::update(double dt)
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
        SPDLOG_INFO("DriveStraightMotion w = ({:.3f}, {:.3f}, {:.3f}, {:.3f})", pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
        const double current_yaw = computeYaw(pose.orientation);
        const foundation::ChassisState state = drive().estimateState();
        SPDLOG_INFO("DriveStraightMotion update: position = ({:.3f}, {:.3f}), yaw = {:.3f} rad, velocity = ({:.3f}, {:.3f}) m/s",
                    pose.position.x(), pose.position.y(), current_yaw,
                    state.vx, state.vy);

        const Eigen::Vector3f v_body(static_cast<float>(state.vx), static_cast<float>(state.vy), 0.0f);
        const Eigen::Vector3f v_world = pose.orientation * v_body;
        const Eigen::Vector2d v_world_xy(static_cast<double>(v_world.x()), static_cast<double>(v_world.y()));
        const Eigen::Vector3f ref_forward_3d = reference_orientation_ * Eigen::Vector3f::UnitX();
        const Eigen::Vector2d ref_forward(ref_forward_3d.x(), ref_forward_3d.y());
        const double ref_forward_norm = ref_forward.norm();
        const Eigen::Vector2d ref_forward_unit = (ref_forward_norm > 1e-6)
                                                     ? (ref_forward / ref_forward_norm)
                                                     : Eigen::Vector2d(1.0, 0.0);
        const double progress_speed = v_world_xy.dot(ref_forward_unit);
        SPDLOG_INFO("DriveStraightMotion progress_speed = {:.3f} m/s", progress_speed);
        distance_travelled_m_ += progress_speed * dt;
        SPDLOG_INFO("DriveStraightMotion distance_travelled = {:.3f} m", distance_travelled_m_);

        const double remaining = cfg_.distance_m - distance_travelled_m_;
        const double remaining_abs = std::abs(remaining);
        SPDLOG_INFO("DriveStraightMotion remaining_distance = {:.3f} m", remaining);

        if (remaining_abs <= cfg_.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            return;
        }

        Eigen::Quaternionf current_orientation = pose.orientation.normalized();

        // Ensure quaternions are in the same hemisphere (q and -q represent the same rotation)
        if (reference_orientation_.dot(current_orientation) < 0.0f)
        {
            current_orientation.coeffs() *= -1.0f;
        }

        // Compute error quaternion: rotation from reference to current
        // This represents how much we've deviated from the reference orientation
        const Eigen::Quaternionf q_error = (current_orientation * reference_orientation_.conjugate()).normalized();
        const double yaw_error = wrapAngle(computeYaw(q_error));
        const double omega_cmd = std::clamp(cfg_.heading_kp * yaw_error, -cfg_.max_heading_rate, cfg_.max_heading_rate);
        SPDLOG_INFO("DriveStraightMotion yaw_error = {:.3f} rad, omega_cmd = {:.3f} rad/s", yaw_error, omega_cmd);

        double vx_cmd = std::clamp(cfg_.distance_kp * remaining, -cfg_.max_speed_mps, cfg_.max_speed_mps);
        SPDLOG_INFO("DriveStraightMotion vx_cmd = {:.3f} m/s", vx_cmd);

        if (std::abs(vx_cmd) < 1e-4)
        {
            const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
            vx_cmd = direction * std::min(cfg_.max_speed_mps, 0.05);
            SPDLOG_INFO("DriveStraightMotion adjusted vx_cmd = {:.3f} m/s", vx_cmd);
        }

        // Apply speed scaling from previous saturation feedback
        vx_cmd *= speed_scale_;
        SPDLOG_INFO("DriveStraightMotion vx_cmd after scaling = {:.3f} m/s (scale={:.3f})", vx_cmd, speed_scale_);

        foundation::ChassisVel cmd{vx_cmd, 0.0, omega_cmd};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust speed scaling based on saturation feedback
        // If motors are saturating AND we have a heading error, reduce speed to leave headroom for correction
        if (motor_cmd.saturated_any && std::abs(yaw_error) > 0.01)
        {
            // Gradually reduce speed scale to leave headroom for heading correction
            speed_scale_ = std::max(cfg_.saturation_derating_factor, speed_scale_ * 0.98);
            SPDLOG_INFO("DriveStraightMotion: Saturation detected with heading error, reducing speed_scale to {:.3f}", speed_scale_);
        }
        else if (!motor_cmd.saturated_any && std::abs(yaw_error) < 0.005)
        {
            // Gradually restore speed when not saturating and heading is good
            speed_scale_ = std::min(1.0, speed_scale_ * 1.02);
            SPDLOG_INFO("DriveStraightMotion: No saturation, increasing speed_scale to {:.3f}", speed_scale_);
        }
    }

    bool DriveStraightMotion::isFinished() const
    {
        return finished_;
    }

    void DriveStraightMotion::complete()
    {
        finished_ = true;
    }

    double DriveStraightMotion::computeYaw(const Eigen::Quaternionf& orientation) const
    {
        const Eigen::Quaternionf q = orientation.normalized();
        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double DriveStraightMotion::wrapAngle(double angle)
    {
        const double two_pi = 2.0 * std::numbers::pi;
        double wrapped = std::fmod(angle + std::numbers::pi, two_pi);
        if (wrapped < 0.0) wrapped += two_pi;
        return wrapped - std::numbers::pi;
    }
}
