#pragma once

#include <memory>
#include <vector>
#include <utility>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/profiled_pid_controller.hpp"
#include "motion/catmull_rom_spline.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    /**
     * Configuration for a spline path motion.
     *
     * Waypoints are in the robot's start frame: first = forward (meters),
     * second = lateral-right (meters, positive = right).
     *
     * If headings_rad is empty, the robot heading follows the spline tangent.
     * If headings_rad has one entry per waypoint, headings are linearly
     * interpolated vs arc-length (omni drivetrains only).
     */
    struct SplineMotionConfig
    {
        std::vector<std::pair<double, double>> waypoints_m;
        std::vector<double> headings_rad;   // empty = tangent-following; one per waypoint = explicit
        double speed_scale{1.0};
    };

    /** Per-cycle diagnostics captured while a SplineMotion instance runs. */
    struct SplineMotionTelemetry
    {
        double time_s{0.0};
        double dt{0.0};
        // Arc-length progress
        double arc_length_m{0.0};           // measured progress along spline
        double arc_target_m{0.0};           // total spline length
        // Path tracking
        double cross_track_m{0.0};          // signed perpendicular distance to path
        // Heading
        double heading_rad{0.0};            // current heading
        double target_heading_rad{0.0};     // desired heading (tangent or interpolated)
        double heading_error_rad{0.0};
        // Velocity
        double filtered_velocity_mps{0.0};
        // Commands
        double cmd_vx_mps{0.0};
        double cmd_vy_mps{0.0};
        double cmd_wz_radps{0.0};
        // Profile state
        double setpoint_position_m{0.0};
        double setpoint_velocity_mps{0.0};
        bool saturated{false};
    };

    /**
     * Closed-loop controller for following a Catmull-Rom spline path.
     *
     * Uses a trapezoidal velocity profile on arc-length for smooth
     * acceleration/deceleration along the path. Heading either tracks
     * the spline tangent (differential and omni) or is independently
     * interpolated between per-waypoint values (omni only).
     */
    class SplineMotion final : public Motion
    {
    public:
        SplineMotion(MotionContext ctx, SplineMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        [[nodiscard]] const std::vector<SplineMotionTelemetry>& getTelemetry() const { return telemetry_; }

    private:
        void complete();

        /** Compute target heading at arc-length s. */
        [[nodiscard]] double targetHeadingAt(double s) const;

        SplineMotionConfig cfg_{};
        std::unique_ptr<CatmullRomSpline> spline_;
        double max_velocity_{0.0};
        ProfiledPIDController profiled_pid_;
        std::unique_ptr<foundation::PidController> heading_pid_;
        bool finished_{false};

        // Heading interpolation LUT: arc-length → heading (built if explicit headings)
        std::vector<double> heading_arc_lengths_;  // arc-length at each point (origin + waypoints)
        std::vector<double> all_headings_;         // heading at each point (origin=0 + user headings)
        bool use_explicit_headings_{false};

        // Projection tracking
        double s_measured_{0.0};  // last projected arc-length (hint for next cycle)

        // Velocity tracking for settling detection
        double prev_s_measured_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};
        static constexpr double kSettlingVelocity{0.02}; // m/s

        // Profile-at-goal counter: complete after N cycles at goal with near-zero setpoint velocity
        int at_goal_cycles_{0};
        static constexpr int kAtGoalSettleCycles{30};  // 30 cycles @ 100Hz = 300ms

        // Saturation feedback
        double speed_scale_{1.0};
        double heading_scale_{1.0};
        int unsaturated_cycles_{0};

        // Telemetry
        double elapsed_time_{0.0};
        std::vector<SplineMotionTelemetry> telemetry_;
    };
}
