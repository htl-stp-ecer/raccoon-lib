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
     */
    struct ArcMotionConfig
    {
        double radius_m{0.0};           // Turning radius (always positive)
        double arc_angle_rad{0.0};      // Total heading change (positive = CCW/left, negative = CW/right)
        double speed_scale{1.0};        // 0-1 fraction of max speed
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

        [[nodiscard]] const std::vector<ArcMotionTelemetry>& getTelemetry() const { return telemetry_; }

    private:
        void complete();

        ArcMotionConfig cfg_{};
        double max_angular_velocity_{0.0};
        ProfiledPIDController profiled_pid_;
        bool finished_{false};

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
