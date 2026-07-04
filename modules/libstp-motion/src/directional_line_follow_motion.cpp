#include "motion/directional_line_follow_motion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/logging.hpp"
#include "motion/motion_pid.hpp"

namespace libstp::motion
{
    DirectionalLineFollowMotion::DirectionalLineFollowMotion(
        MotionContext ctx,
        DirectionalLineFollowMotionConfig config)
        : Motion(ctx)
        , cfg_(config)
    {
    }

    void DirectionalLineFollowMotion::captureInitialPose()
    {
        const auto pose = odometry().getPose();
        initial_position_m_ = Eigen::Vector2d(
            static_cast<double>(pose.position.x()),
            static_cast<double>(pose.position.y()));
        initial_heading_rad_ = static_cast<double>(pose.heading);
    }

    double DirectionalLineFollowMotion::straightDistanceFromStart() const
    {
        const auto pose = odometry().getPose();
        const double dx = static_cast<double>(pose.position.x()) - initial_position_m_.x();
        const double dy = static_cast<double>(pose.position.y()) - initial_position_m_.y();
        return std::hypot(dx, dy);
    }

    void DirectionalLineFollowMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        max_linear_mps_ = ctx_.pid_config.linear.max_velocity;
        max_lateral_mps_ = ctx_.pid_config.lateral.max_velocity;
        vx_mps_ = cfg_.heading_speed * max_linear_mps_;
        vy_mps_ = cfg_.strafe_speed * max_lateral_mps_;
        sensor_error_ = 0.0;

        captureInitialPose();
        drive().resetVelocityControllers();

        correction_pid_ = std::make_unique<foundation::PidController>(
            foundation::PidConfig{
                cfg_.kp,
                cfg_.ki,
                cfg_.kd,
                1.0,
                -1.0,
                1.0,
            });

        heading_pid_.reset();
        if (cfg_.correction_mode != LineFollowCorrectionMode::Angular && cfg_.heading_hold)
        {
            const auto& h = ctx_.pid_config.heading;
            heading_pid_ = std::make_unique<foundation::PidController>(
                foundation::PidConfig{
                    h.kp,
                    h.ki,
                    h.kd,
                    1.0,
                    -1.0,
                    1.0,
                });
        }

        LIBSTP_LOG_TRACE(
            "DirectionalLineFollowMotion started: heading={:.3f}, strafe={:.3f}, correction_mode={}",
            cfg_.heading_speed,
            cfg_.strafe_speed,
            static_cast<int>(cfg_.correction_mode));
    }

    void DirectionalLineFollowMotion::complete()
    {
        finished_ = true;
        drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
    }

    void DirectionalLineFollowMotion::setSensorError(double error)
    {
        sensor_error_ = error;
    }

    void DirectionalLineFollowMotion::update(double dt)
    {
        if (!started_) start();

        if (finished_)
        {
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            if (dt > 0.0)
            {
                [[maybe_unused]] const auto motor_cmd = drive().update(dt);
            }
            return;
        }

        if (dt <= 0.0)
        {
            LIBSTP_LOG_WARN(
                "DirectionalLineFollowMotion::update called with invalid dt={:.6f}s (must be > 0)",
                dt);
            return;
        }

        elapsed_time_ += dt;
        odometry().update(dt);

        const double straight_distance = straightDistanceFromStart();
        if (cfg_.has_distance_target && straight_distance >= cfg_.distance_m)
        {
            complete();
            [[maybe_unused]] const auto motor_cmd = drive().update(dt);
            telemetry_.push_back(DirectionalLineFollowMotionTelemetry{
                .time_s = elapsed_time_,
                .dt = dt,
                .error = sensor_error_,
                .correction = 0.0,
                .heading_error_rad = 0.0,
                .heading_speed_mps = vx_mps_,
                .strafe_speed_mps = vy_mps_,
                .cmd_vx_mps = 0.0,
                .cmd_vy_mps = 0.0,
                .cmd_wz_radps = 0.0,
                .straight_distance_m = straight_distance,
                .finished = true,
            });
            return;
        }

        const double correction = correction_pid_->update(sensor_error_, dt) * cfg_.correction_sign;
        double heading_error = 0.0;
        double wz = 0.0;
        double vx = vx_mps_;
        double vy = vy_mps_;

        // The heading to hold: an absolute world target when the caller pinned
        // one (has_target_heading), otherwise the heading captured at start.
        const double hold_heading_rad =
            cfg_.has_target_heading ? cfg_.target_heading_rad : initial_heading_rad_;

        if (cfg_.correction_mode == LineFollowCorrectionMode::Forward)
        {
            vx += correction * max_linear_mps_;
            if (cfg_.heading_hold)
            {
                heading_error = std::remainder(
                    hold_heading_rad - odometry().getHeading(),
                    2.0 * std::numbers::pi);
                wz = heading_pid_->update(heading_error, dt);
            }
        }
        else if (cfg_.correction_mode == LineFollowCorrectionMode::Lateral)
        {
            vy += correction * max_lateral_mps_;
            if (cfg_.heading_hold)
            {
                heading_error = std::remainder(
                    hold_heading_rad - odometry().getHeading(),
                    2.0 * std::numbers::pi);
                wz = heading_pid_->update(heading_error, dt);
            }
        }
        else
        {
            wz = correction;
        }

        foundation::ChassisVelocity cmd{vx, vy, wz};
        drive().setVelocity(cmd);
        [[maybe_unused]] const auto motor_cmd = drive().update(dt);

        // Per-tick breakdown of how the sensor error turns into a chassis
        // command. Trace level (compiled out unless LIBSTP_TRACE_LOGGING) so the
        // 100 Hz spam only appears when explicitly debugging line-follow.
        LIBSTP_LOG_TRACE(
            "lf tick: mode={} err={:.4f} corr={:.4f} -> vx={:.4f} vy={:.4f} wz={:.4f} "
            "(base vx={:.4f} vy={:.4f}, heading_err={:.4f}rad, dt={:.4f})",
            static_cast<int>(cfg_.correction_mode),
            sensor_error_,
            correction,
            vx,
            vy,
            wz,
            vx_mps_,
            vy_mps_,
            heading_error,
            dt);

        if (telemetry_.size() >= kMaxTelemetrySamples)
        {
            telemetry_.erase(telemetry_.begin(), telemetry_.begin() + static_cast<long>(kMaxTelemetrySamples / 2));
        }
        telemetry_.push_back(DirectionalLineFollowMotionTelemetry{
            .time_s = elapsed_time_,
            .dt = dt,
            .error = sensor_error_,
            .correction = correction,
            .heading_error_rad = heading_error,
            .heading_speed_mps = vx_mps_,
            .strafe_speed_mps = vy_mps_,
            .cmd_vx_mps = vx,
            .cmd_vy_mps = vy,
            .cmd_wz_radps = wz,
            .straight_distance_m = straight_distance,
            .finished = finished_,
        });
    }

    bool DirectionalLineFollowMotion::isFinished() const
    {
        return finished_;
    }
}
