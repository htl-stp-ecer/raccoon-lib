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
        cfg_.saturation_derating_factor = std::clamp(cfg_.saturation_derating_factor, 0.1, 0.99);
        cfg_.saturation_min_speed_scale = std::clamp(cfg_.saturation_min_speed_scale, 0.01, 1.0);
        cfg_.saturation_recovery_rate = std::clamp(cfg_.saturation_recovery_rate, 0.0, 0.5);
        cfg_.saturation_heading_error_rad = std::clamp(cfg_.saturation_heading_error_rad, 0.0, 0.5);
        cfg_.heading_saturation_derating_factor = std::clamp(cfg_.heading_saturation_derating_factor, 0.1, 0.99);
        cfg_.heading_min_scale = std::clamp(cfg_.heading_min_scale, 0.05, 1.0);
        cfg_.heading_recovery_rate = std::clamp(cfg_.heading_recovery_rate, 0.0, 0.5);
        cfg_.heading_recovery_error_rad = std::clamp(cfg_.heading_recovery_error_rad, 0.0, 0.5);
    }

    void DriveStraightMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        distance_travelled_m_ = 0.0;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        reorienting_ = false;

        const auto pose = odometry().getPose();
        reference_orientation_ = pose.orientation.normalized();
        reference_position_ = pose.position;  // Store starting position for lateral tracking
        const double reference_yaw = computeYaw(reference_orientation_);
        SPDLOG_INFO("DriveStraightMotion started: target_distance = {:.3f} m, max_speed = {:.3f} m/s, reference_yaw = {:.3f} rad, start_pos = ({:.3f}, {:.3f})",
                    cfg_.distance_m, cfg_.max_speed_mps, reference_yaw, reference_position_.x(), reference_position_.y());
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

        // ===== COMPUTE LATERAL DRIFT =====
        // Calculate the desired path direction in world frame (reuse ref_forward_3d from above)
        const Eigen::Vector2f ref_forward_2d(ref_forward_3d.x(), ref_forward_3d.y());
        const Eigen::Vector2f ref_forward_unit_2f = ref_forward_2d.normalized();

        // Position error from reference starting position
        const Eigen::Vector3f position_error_world = pose.position - reference_position_;
        const Eigen::Vector2f position_error_2d(position_error_world.x(), position_error_world.y());

        // Project onto reference direction to get forward progress
        const float forward_progress = position_error_2d.dot(ref_forward_unit_2f);

        // Compute lateral error (perpendicular to path)
        const Eigen::Vector2f lateral_direction(-ref_forward_unit_2f.y(), ref_forward_unit_2f.x());  // 90° rotation
        const float lateral_error_world = position_error_2d.dot(lateral_direction);

        // Transform lateral error to body frame
        const Eigen::Quaternionf current_orientation = pose.orientation.normalized();
        const Eigen::Vector3f lateral_error_world_3d(lateral_error_world * lateral_direction.x(),
                                                      lateral_error_world * lateral_direction.y(),
                                                      0.0f);
        const Eigen::Vector3f lateral_error_body_3d = current_orientation.inverse() * lateral_error_world_3d;
        const double lateral_error_body = static_cast<double>(lateral_error_body_3d.y());  // Body-frame y is lateral

        SPDLOG_INFO("DriveStraightMotion lateral_error_world = {:.3f} m, lateral_error_body = {:.3f} m",
                    lateral_error_world, lateral_error_body);

        // ===== HEADING CONTROL =====
        // Ensure quaternions are in the same hemisphere (q and -q represent the same rotation)
        Eigen::Quaternionf current_orientation_corrected = current_orientation;
        if (reference_orientation_.dot(current_orientation_corrected) < 0.0f)
        {
            current_orientation_corrected.coeffs() *= -1.0f;
        }

        // Compute error quaternion: rotation from reference to current
        const Eigen::Quaternionf q_error = (current_orientation_corrected * reference_orientation_.conjugate()).normalized();
        double yaw_error = wrapAngle(computeYaw(q_error));

        // Check if kinematics supports lateral motion
        const bool supports_lateral = drive().getKinematics().supportsLateralMotion();

        // ===== LATERAL CORRECTION STRATEGY =====
        double vy_cmd = 0.0;
        double omega_cmd = 0.0;

        if (supports_lateral)
        {
            // MECANUM STRATEGY: Direct lateral correction using vy
            vy_cmd = std::clamp(cfg_.lateral_kp * lateral_error_body, -cfg_.max_speed_mps * 0.5, cfg_.max_speed_mps * 0.5);
            omega_cmd = std::clamp(cfg_.heading_kp * yaw_error, -cfg_.max_heading_rate, cfg_.max_heading_rate);
            SPDLOG_INFO("DriveStraightMotion [MECANUM] vy_cmd = {:.3f} m/s, omega_cmd = {:.3f} rad/s", vy_cmd, omega_cmd);
        }
        else
        {
            // DIFFERENTIAL STRATEGY: Combined approach (bias + stop-and-reorient)
            const double lateral_error_abs = std::abs(lateral_error_world);

            if (lateral_error_abs > cfg_.lateral_reorient_threshold_m && !reorienting_)
            {
                // Large lateral error: enter reorientation mode
                reorienting_ = true;
                SPDLOG_INFO("DriveStraightMotion [DIFFERENTIAL] Large lateral error ({:.3f} m), entering reorientation mode", lateral_error_abs);
            }

            if (reorienting_)
            {
                // Stop and reorient: compute desired heading that points back to path
                // Target position is on the reference line at current forward progress
                const Eigen::Vector2f target_position_2d = ref_forward_unit_2f * forward_progress;
                const Eigen::Vector2f to_target = target_position_2d - position_error_2d;

                if (to_target.norm() > 0.01f)
                {
                    // Compute desired heading in world frame
                    const float desired_yaw_world = std::atan2(to_target.y(), to_target.x());
                    const double current_yaw_world = computeYaw(current_orientation);
                    double yaw_to_target = wrapAngle(desired_yaw_world - current_yaw_world);

                    omega_cmd = std::clamp(cfg_.heading_kp * yaw_to_target, -cfg_.max_heading_rate, cfg_.max_heading_rate);
                    SPDLOG_INFO("DriveStraightMotion [DIFFERENTIAL-REORIENT] yaw_to_target = {:.3f} rad, omega_cmd = {:.3f} rad/s",
                                yaw_to_target, omega_cmd);

                    // Exit reorientation when aligned and lateral error is small
                    if (std::abs(yaw_to_target) < 0.1 && lateral_error_abs < cfg_.lateral_reorient_threshold_m * 0.7)
                    {
                        reorienting_ = false;
                        SPDLOG_INFO("DriveStraightMotion [DIFFERENTIAL] Exiting reorientation mode");
                    }
                }
                else
                {
                    // Already on target, exit reorientation
                    reorienting_ = false;
                }
            }
            else
            {
                // Normal mode: bias heading slightly to correct lateral drift
                const double heading_bias = std::atan(cfg_.lateral_heading_bias_gain * lateral_error_world);
                const double biased_yaw_error = yaw_error + heading_bias;
                omega_cmd = std::clamp(cfg_.heading_kp * biased_yaw_error, -cfg_.max_heading_rate, cfg_.max_heading_rate);
                SPDLOG_INFO("DriveStraightMotion [DIFFERENTIAL-BIAS] heading_bias = {:.3f} rad, biased_yaw_error = {:.3f} rad, omega_cmd = {:.3f} rad/s",
                            heading_bias, biased_yaw_error, omega_cmd);
            }
        }

        // ===== FORWARD VELOCITY CONTROL =====
        double vx_cmd = std::clamp(cfg_.distance_kp * remaining, -cfg_.max_speed_mps, cfg_.max_speed_mps);

        if (std::abs(vx_cmd) < 1e-4)
        {
            const double direction = (remaining >= 0.0) ? 1.0 : -1.0;
            vx_cmd = direction * std::min(cfg_.max_speed_mps, 0.05);
        }

        // Reduce forward speed during reorientation (differential only)
        if (reorienting_)
        {
            vx_cmd *= 0.3;  // Slow down to 30% during reorientation
            SPDLOG_INFO("DriveStraightMotion [DIFFERENTIAL-REORIENT] Reducing vx_cmd to {:.3f} m/s", vx_cmd);
        }

        // Apply scaling from previous saturation feedback (reduce translation first, then heading if needed)
        vx_cmd *= speed_scale_;
        vy_cmd *= speed_scale_;
        const double omega_cmd_scaled = omega_cmd * heading_scale_;
        SPDLOG_INFO(
            "DriveStraightMotion scaled cmd: vx = {:.3f} m/s, vy = {:.3f} m/s, omega = {:.3f} rad/s (speed_scale={:.3f}, heading_scale={:.3f})",
            vx_cmd,
            vy_cmd,
            omega_cmd_scaled,
            speed_scale_,
            heading_scale_);

        // ===== SEND COMMANDS =====
        foundation::ChassisVel cmd{vx_cmd, vy_cmd, omega_cmd_scaled};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Adjust scaling based on saturation feedback
        const double yaw_error_abs = std::abs(yaw_error);
        if (motor_cmd.saturated_any && yaw_error_abs > cfg_.saturation_heading_error_rad)
        {
            const double prev_speed_scale = speed_scale_;
            const double prev_heading_scale = heading_scale_;

            if (speed_scale_ > cfg_.saturation_min_speed_scale + 1e-6)
            {
                speed_scale_ = std::max(
                    cfg_.saturation_min_speed_scale,
                    speed_scale_ * cfg_.saturation_derating_factor);
            }
            else
            {
                heading_scale_ = std::max(
                    cfg_.heading_min_scale,
                    heading_scale_ * cfg_.heading_saturation_derating_factor);
            }

            SPDLOG_INFO(
                "DriveStraightMotion: Saturation detected (mask=0x{:X}, yaw_error={:.3f}) -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                motor_cmd.saturation_mask,
                yaw_error,
                prev_speed_scale,
                speed_scale_,
                prev_heading_scale,
                heading_scale_);
        }
        else if (!motor_cmd.saturated_any && yaw_error_abs < cfg_.heading_recovery_error_rad)
        {
            const double prev_speed_scale = speed_scale_;
            const double prev_heading_scale = heading_scale_;

            speed_scale_ = std::min(1.0, speed_scale_ + cfg_.saturation_recovery_rate);
            heading_scale_ = std::min(1.0, heading_scale_ + cfg_.heading_recovery_rate);

            if (prev_speed_scale != speed_scale_ || prev_heading_scale != heading_scale_)
            {
                SPDLOG_INFO(
                    "DriveStraightMotion: Recovery -> speed_scale {:.3f}->{:.3f}, heading_scale {:.3f}->{:.3f}",
                    prev_speed_scale,
                    speed_scale_,
                    prev_heading_scale,
                    heading_scale_);
            }
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
