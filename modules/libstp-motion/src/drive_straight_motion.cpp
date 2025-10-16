#include "motion/drive_straight_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/types.hpp"

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

        const auto orientation = imu().getOrientation();
        reference_yaw_ = computeYaw(orientation);
    }

    void DriveStraightMotion::update(double dt)
    {
        if (!started_) start();

        if (finished_)
        {
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            if (dt > 0.0) drive().update(dt);
            return;
        }

        if (dt <= 0.0)
        {
            return;
        }

        const auto orientation = imu().getOrientation();
        const double current_yaw = computeYaw(orientation);
        const foundation::ChassisState state = drive().estimateState();

        const Eigen::Vector3f v_body(static_cast<float>(state.vx), static_cast<float>(state.vy), 0.0f);
        const Eigen::Vector3f v_world = orientation * v_body;
        const Eigen::Vector2d v_world_xy(static_cast<double>(v_world.x()), static_cast<double>(v_world.y()));
        const Eigen::Vector2d ref_forward(std::cos(reference_yaw_), std::sin(reference_yaw_));
        const double progress_speed = v_world_xy.dot(ref_forward);
        distance_travelled_m_ += progress_speed * dt;

        const double remaining = cfg_.distance_m - distance_travelled_m_;
        const double remaining_abs = std::abs(remaining);

        if (remaining_abs <= cfg_.distance_tolerance_m)
        {
            complete();
            drive().setVelocity(foundation::ChassisVel{0.0, 0.0, 0.0});
            drive().update(dt);
            return;
        }

        const double yaw_error = wrapAngle(reference_yaw_ - current_yaw);
        const double omega_cmd = std::clamp(cfg_.heading_kp * yaw_error, -cfg_.max_heading_rate, cfg_.max_heading_rate);

        double vx_cmd = std::clamp(cfg_.distance_kp * remaining, -cfg_.max_speed_mps, cfg_.max_speed_mps);

        if (std::abs(vx_cmd) < 1e-4)
        {
            const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
            vx_cmd = direction * std::min(cfg_.max_speed_mps, 0.05);
        }

        foundation::ChassisVel cmd{vx_cmd, 0.0, omega_cmd};
        drive().setVelocity(cmd);
        drive().update(dt);
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
