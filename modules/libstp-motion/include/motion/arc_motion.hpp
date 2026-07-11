#pragma once

#include <memory>
#include <vector>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/profiled_pid_controller.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    /**
     * Configuration for a circular arc motion.
     *
     * The robot drives forward while simultaneously turning, tracing a circular
     * arc of the given radius. The arc is complete when the heading has changed
     * by `arc_angle_rad`.
     *
     * - `arc_angle_rad > 0` → counter-clockwise (left) arc
     * - `arc_angle_rad < 0` → clockwise (right) arc
     * - `radius_m` is always positive (distance from arc center to robot)
     *
     * `speed_scale` carries both magnitude and travel direction:
     * - `speed_scale > 0` → drive the arc forwards
     * - `speed_scale < 0` → drive the arc BACKWARDS along the *same* circle: the
     *   arc centre stays on the same side, the robot reverses and its heading
     *   turns the opposite way (a forward left arc's mirror in time). The
     *   effective magnitude is `|speed_scale|`, clamped to [0.01, 1.0].
     */
    struct ArcMotionConfig
    {
        double radius_m{0.0};           // Turning radius (always positive)
        double arc_angle_rad{0.0};      // Total heading change (positive = CCW/left, negative = CW/right)
        double speed_scale{1.0};        // fraction of max speed; sign = travel direction (<0 = reverse)
        bool lateral{false};            // true = strafe arc (vy), false = drive arc (vx)
    };

    /** Per-cycle diagnostics captured while an `ArcMotion` instance runs. */
    struct ArcMotionTelemetry
    {
        double time_s{0.0};
        double dt{0.0};
        // Heading tracking
        double target_angle_rad{0.0};
        double heading_rad{0.0};
        double heading_error_rad{0.0};
        // Arc progress
        double arc_position_m{0.0};     // distance traveled along arc
        double arc_target_m{0.0};       // total arc length
        // Velocity
        double filtered_velocity_radps{0.0};
        // Commands
        double cmd_vx_mps{0.0};
        double cmd_vy_mps{0.0};
        double cmd_wz_radps{0.0};
        // PID internals
        double pid_raw{0.0};
        double setpoint_position_rad{0.0};
        double setpoint_velocity_radps{0.0};
        bool saturated{false};
    };

    /**
     * Closed-loop controller for driving along a circular arc.
     *
     * Uses a profiled PID on heading (like TurnMotion) and derives the forward
     * velocity from the angular velocity command: `vx = |omega| * radius`.
     * This produces coordinated acceleration/deceleration along the arc.
     *
     * The max angular velocity is limited to ensure the derived forward velocity
     * does not exceed the linear axis limits.
     */
    class ArcMotion final : public Motion
    {
    public:
        ArcMotion(MotionContext ctx, ArcMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        /**
         * Begin arc from current state without resetting odometry.
         *
         * @param heading_offset_rad  Current heading (subtracted from reads)
         * @param initial_angular_velocity  Current angular velocity to seed the profile with
         */
        void startWarm(double heading_offset_rad, double initial_angular_velocity);

        /** When true, complete() sets finished_ without calling hardStop(). */
        void setSuppressHardStopOnComplete(bool suppress) { suppress_hard_stop_ = suppress; }

        /** Check if target arc angle is reached, ignoring velocity settling. */
        [[nodiscard]] bool hasReachedAngle() const;

        /** Current filtered angular velocity (rad/s). */
        [[nodiscard]] double getFilteredVelocity() const { return filtered_velocity_; }

        [[nodiscard]] const std::vector<ArcMotionTelemetry>& getTelemetry() const { return telemetry_; }

    private:
        void complete();

        ArcMotionConfig cfg_{};
        // Travel direction from the sign of speed_scale: +1 forwards, -1 reverse.
        double travel_dir_{1.0};
        // Heading target actually tracked by the PID. Equals cfg_.arc_angle_rad
        // forwards; negated for a reverse arc so the robot turns the opposite way
        // while the arc centre stays on the same side.
        double goal_angle_rad_{0.0};
        double max_angular_velocity_{0.0};
        ProfiledPIDController profiled_pid_;
        bool finished_{false};
        bool suppress_hard_stop_{false};
        double heading_offset_rad_{0.0};

        // Absolute heading at start; subtracted from current absolute heading to
        // produce the body-frame arc progress. Replaces the historical
        // odometry-reset semantics.
        double initial_absolute_heading_rad_{0.0};

        // Velocity tracking for settling detection
        double prev_heading_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};
        static constexpr double kSettlingVelocity{0.05}; // rad/s

        // Telemetry
        double elapsed_time_{0.0};
        std::vector<ArcMotionTelemetry> telemetry_;
        int cycle_{0};
    };
}
