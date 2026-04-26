#include "motion/spline_motion.hpp"
#include "motion/motion_pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "foundation/logging.hpp"

namespace libstp::motion
{
    static ProfiledPIDController makeSplineProfiledPID(
        const UnifiedMotionPidConfig& pid_config,
        double max_velocity)
    {
        ProfiledPIDController::Config cfg;
        cfg.pid = pid_config.distance;
        cfg.velocity_ff = pid_config.velocity_ff;

        TrapezoidalProfile::Constraints constraints;
        constraints.max_velocity = max_velocity;
        constraints.max_acceleration = pid_config.linear.acceleration;
        constraints.max_deceleration = pid_config.linear.deceleration;

        return ProfiledPIDController(cfg, constraints);
    }

    static double computeMaxVelocity(const UnifiedMotionPidConfig& pid_config,
                                      double speed_scale)
    {
        double scale = std::clamp(speed_scale, 0.01, 1.0);
        return scale * pid_config.linear.max_velocity;
    }

    SplineMotion::SplineMotion(MotionContext ctx, SplineMotionConfig config)
        : Motion(ctx), cfg_(std::move(config))
        , max_velocity_(computeMaxVelocity(ctx_.pid_config, cfg_.speed_scale))
        , profiled_pid_(makeSplineProfiledPID(ctx_.pid_config, max_velocity_))
    {
        heading_pid_ = createPidController(ctx_.pid_config, PidType::Heading);
    }

    void SplineMotion::start()
    {
        if (started_) return;
        started_ = true;
        finished_ = false;
        speed_scale_ = 1.0;
        heading_scale_ = 1.0;
        unsaturated_cycles_ = 0;
        off_path_warned_ = false;
        at_goal_cycles_ = 0;
        s_measured_ = 0.0;
        prev_s_measured_ = 0.0;
        filtered_velocity_ = 0.0;
        elapsed_time_ = 0.0;
        telemetry_.clear();

        // Build spline from waypoints, prepending the origin (robot starts here)
        std::vector<Eigen::Vector2d> points;
        points.reserve(cfg_.waypoints_m.size() + 1);
        points.emplace_back(0.0, 0.0);  // robot's start position
        for (const auto& [x, y] : cfg_.waypoints_m)
        {
            points.emplace_back(x, y);
        }
        spline_ = std::make_unique<CatmullRomSpline>(std::move(points));

        // Absolute heading mode: capture IMU heading before reset so it can
        // be added to relative target headings during update().
        use_absolute_heading_ = cfg_.use_absolute_heading;
        initial_absolute_heading_rad_ = use_absolute_heading_
            ? odometry().getAbsoluteHeading()
            : 0.0;

        // Set up heading interpolation if explicit headings provided
        use_explicit_headings_ = !cfg_.headings_rad.empty();
        if (use_explicit_headings_)
        {
            // Build combined headings: origin heading (0) + user-specified headings
            all_headings_.reserve(cfg_.headings_rad.size() + 1);
            all_headings_.push_back(0.0);  // robot starts facing its initial heading
            all_headings_.insert(all_headings_.end(),
                                 cfg_.headings_rad.begin(), cfg_.headings_rad.end());

            // Compute arc-length at each waypoint: origin + user waypoints
            heading_arc_lengths_.resize(all_headings_.size());
            heading_arc_lengths_[0] = 0.0;  // origin

            for (size_t i = 0; i < cfg_.waypoints_m.size(); ++i)
            {
                Eigen::Vector2d wp(cfg_.waypoints_m[i].first, cfg_.waypoints_m[i].second);
                double hint = heading_arc_lengths_[i];
                heading_arc_lengths_[i + 1] = spline_->findNearestArcLength(wp, hint);
            }
            // Ensure last waypoint maps to total length
            heading_arc_lengths_.back() = spline_->totalLength();
        }
        else if (cfg_.final_heading_rad.has_value())
        {
            // Simple two-point LUT: start = 0 (robot start heading), end = final_heading_rad.
            // targetHeadingAt() interpolates linearly by arc-length between them.
            use_explicit_headings_ = true;
            all_headings_ = {0.0, cfg_.final_heading_rad.value()};
            heading_arc_lengths_ = {0.0, spline_->totalLength()};
        }

        odometry().reset();
        heading_pid_->reset();

        // Profile: start at s=0, goal = total arc length
        profiled_pid_.reset(0.0);
        profiled_pid_.setGoal(spline_->totalLength());

        LIBSTP_LOG_TRACE("SplineMotion started: {} waypoints, arc_length={:.3f} m, max_velocity={:.3f} m/s, heading_mode={}, abs_heading={}",
                    cfg_.waypoints_m.size(), spline_->totalLength(), max_velocity_,
                    (use_explicit_headings_ ? "explicit" : "tangent"),
                    (use_absolute_heading_ ? "absolute" : "relative"));
    }

    void SplineMotion::update(double dt)
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
            LIBSTP_LOG_WARN("SplineMotion::update called with invalid dt={:.6f}s", dt);
            return;
        }

        elapsed_time_ += dt;
        odometry().update(dt);

        // Get robot position in start frame
        const auto dist = odometry().getDistanceFromOrigin();
        const Eigen::Vector2d robot_pos(dist.forward, dist.lateral);
        const double current_heading = use_absolute_heading_
            ? odometry().getAbsoluteHeading()
            : odometry().getHeading();

        // Project robot position onto spline.
        // Rate-limit backward jumps: allow at most max_velocity * dt backward per cycle.
        // This prevents the findNearestArcLength oscillation when past the endpoint
        // (where it alternates between s=0 and s=totalLength) while still allowing
        // genuine small backward corrections from tracking errors.
        double s_new = spline_->findNearestArcLength(robot_pos, s_measured_);
        double max_backward = max_velocity_ * dt * 2.0;  // generous margin
        s_measured_ = std::max(s_measured_ - max_backward, s_new);

        // Filtered velocity for settling detection
        const double raw_velocity = (s_measured_ - prev_s_measured_) / dt;
        filtered_velocity_ = kVelocityFilterAlpha * raw_velocity
                           + (1.0 - kVelocityFilterAlpha) * filtered_velocity_;
        prev_s_measured_ = s_measured_;

        const double arc_error = spline_->totalLength() - s_measured_;

        // Profiled PID on arc-length → speed command along path
        double speed_cmd_raw = profiled_pid_.calculate(s_measured_, dt);
        double speed_cmd = std::clamp(speed_cmd_raw, -max_velocity_, max_velocity_);
        speed_cmd *= speed_scale_;

        // Get the profiled setpoint arc-length for heading/tangent evaluation
        const auto sp = profiled_pid_.getSetpoint();
        const double s_setpoint = std::clamp(sp.position, 0.0, spline_->totalLength());

        // Check settling: near end of path AND nearly stopped
        if (std::abs(arc_error) <= ctx_.pid_config.distance_tolerance_m &&
            std::abs(filtered_velocity_) < kSettlingVelocity)
        {
            LIBSTP_LOG_TRACE("SplineMotion completed: s={:.3f}/{:.3f} m, filt_vel={:.4f} m/s",
                        s_measured_, spline_->totalLength(), filtered_velocity_);
            complete();
            drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
            const auto motor_cmd = drive().update(dt);
            return;
        }

        // Secondary settling: if the profiled PID has been at its goal with near-zero
        // velocity for many cycles, complete even if s_measured is far from totalLength
        // (handles the case where the robot overshoots past the spline and can't project back)
        if (sp.position >= spline_->totalLength() - ctx_.pid_config.distance_tolerance_m
            && std::abs(sp.velocity) < 0.01)
        {
            ++at_goal_cycles_;
            if (at_goal_cycles_ >= kAtGoalSettleCycles)
            {
                LIBSTP_LOG_TRACE("SplineMotion completed (at-goal timeout): s={:.3f}/{:.3f} m, goal_cycles={}",
                            s_measured_, spline_->totalLength(), at_goal_cycles_);
                complete();
                drive().setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
                const auto motor_cmd = drive().update(dt);
                return;
            }
        }
        else
        {
            at_goal_cycles_ = 0;
        }

        // Heading target: targetHeadingAt() returns a heading relative to the
        // robot's start orientation.  In absolute mode we offset by the captured
        // IMU heading so the error is computed in the absolute frame.
        const double target_heading = targetHeadingAt(s_setpoint)
                                    + initial_absolute_heading_rad_;

        // Heading error
        const double heading_error = std::remainder(target_heading - current_heading,
                                                     2.0 * std::numbers::pi);

        // Heading PID → angular velocity command
        double omega_cmd_raw = heading_pid_->update(heading_error, dt);
        double omega_cmd = omega_cmd_raw * heading_scale_;

        // Decompose speed along path into robot-frame velocity.
        // Spline frame: x=forward, y=right-positive. Heading convention: positive=CCW.
        // Convert by negating y for atan2 (left-handed frame → right-handed heading).
        Eigen::Vector2d tangent = spline_->tangentAt(s_setpoint);
        double path_angle = std::atan2(-tangent.y(), tangent.x());
        double rel_angle = path_angle - current_heading;

        double vx = speed_cmd * std::cos(rel_angle);
        double vy = speed_cmd * std::sin(rel_angle);

        foundation::ChassisVelocity cmd{vx, vy, omega_cmd};
        drive().setVelocity(cmd);
        const auto motor_cmd = drive().update(dt);

        // Cross-track error (signed perpendicular distance)
        Eigen::Vector2d path_point = spline_->positionAt(s_measured_);
        Eigen::Vector2d diff = robot_pos - path_point;
        // Signed: positive = robot is to the right of the path
        Eigen::Vector2d path_tangent = spline_->tangentAt(s_measured_);
        double cross_track = -diff.x() * path_tangent.y() + diff.y() * path_tangent.x();

        const bool off_path = std::abs(cross_track) > kOffPathThreshold;
        if (off_path && !off_path_warned_)
        {
            LIBSTP_LOG_WARN("SplineMotion: cross-track error {:.3f} m exceeds threshold at s={:.3f}/{:.3f} m",
                cross_track, s_measured_, spline_->totalLength());
            off_path_warned_ = true;
        }

        // Record telemetry
        telemetry_.push_back(SplineMotionTelemetry{
            .time_s = elapsed_time_,
            .dt = dt,
            .arc_length_m = s_measured_,
            .arc_target_m = spline_->totalLength(),
            .cross_track_m = cross_track,
            .heading_rad = current_heading,
            .target_heading_rad = target_heading,
            .heading_error_rad = heading_error,
            .filtered_velocity_mps = filtered_velocity_,
            .cmd_vx_mps = vx,
            .cmd_vy_mps = vy,
            .cmd_wz_radps = omega_cmd,
            .setpoint_position_m = sp.position,
            .setpoint_velocity_mps = sp.velocity,
            .saturated = motor_cmd.saturated_any,
            .off_path = off_path,
        });

        // Saturation feedback with hysteresis (same pattern as LinearMotion)
        const double heading_error_abs = std::abs(heading_error);
        if (motor_cmd.saturated_any && heading_error_abs > ctx_.pid_config.heading_saturation_error_rad)
        {
            unsaturated_cycles_ = 0;

            if (speed_scale_ > ctx_.pid_config.saturation_min_scale + 1e-6)
            {
                speed_scale_ = std::max(
                    ctx_.pid_config.saturation_min_scale,
                    speed_scale_ * ctx_.pid_config.saturation_derating_factor);
            }
            else
            {
                heading_scale_ = std::max(
                    ctx_.pid_config.heading_min_scale,
                    heading_scale_ * ctx_.pid_config.heading_saturation_derating_factor);
            }
        }
        else if (!motor_cmd.saturated_any && heading_error_abs < ctx_.pid_config.heading_recovery_error_rad)
        {
            ++unsaturated_cycles_;

            const bool can_recover = unsaturated_cycles_ >= ctx_.pid_config.saturation_hold_cycles;
            const bool needs_recovery = speed_scale_ < ctx_.pid_config.saturation_recovery_threshold ||
                                        heading_scale_ < ctx_.pid_config.saturation_recovery_threshold;

            if (can_recover && needs_recovery)
            {
                speed_scale_ = std::min(1.0, speed_scale_ + ctx_.pid_config.saturation_recovery_rate);
                heading_scale_ = std::min(1.0, heading_scale_ + ctx_.pid_config.heading_recovery_rate);
            }
        }
    }

    bool SplineMotion::isFinished() const
    {
        return finished_;
    }

    double SplineMotion::targetHeadingAt(double s) const
    {
        if (!use_explicit_headings_)
        {
            // Tangent-following: heading = direction of tangent vector.
            // Negate y because spline uses right-positive lateral but heading is CCW-positive.
            Eigen::Vector2d t = spline_->tangentAt(s);
            return std::atan2(-t.y(), t.x());
        }

        // Explicit heading interpolation vs arc-length
        const auto& arcs = heading_arc_lengths_;
        const auto& headings = all_headings_;

        if (s <= arcs.front()) return headings.front();
        if (s >= arcs.back()) return headings.back();

        // Find the segment [i, i+1] that contains s
        auto it = std::upper_bound(arcs.begin(), arcs.end(), s);
        int idx = static_cast<int>(it - arcs.begin());
        if (idx == 0) idx = 1;
        if (idx >= static_cast<int>(arcs.size())) idx = static_cast<int>(arcs.size()) - 1;

        double s0 = arcs[idx - 1];
        double s1 = arcs[idx];
        double frac = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;

        // Interpolate heading, handling wraparound
        double h0 = headings[idx - 1];
        double h1 = headings[idx];
        double diff = std::remainder(h1 - h0, 2.0 * std::numbers::pi);
        return h0 + frac * diff;
    }

    void SplineMotion::complete()
    {
        finished_ = true;
        drive().hardStop();
    }
}
