#pragma once

#include "motion/motion.hpp"
#include "motion/profiled_pid_controller.hpp"

namespace libstp::motion
{
    /** Configuration for an in-place turn. */
    struct TurnConfig
    {
        double target_angle_rad{0.0};        // Target turn angle (positive = CCW, negative = CW)
        double speed_scale{1.0};             // 0-1 fraction of AxisConstraints.angular.max_velocity
        double kS{0.0};                      // Static friction compensation (rad/s)
    };

    /**
     * Profiled PID turn controller.
     *
     * Uses a TrapezoidalProfile to generate smooth angular setpoints,
     * then PID tracks those setpoints with velocity feedforward.
     * Supports asymmetric acceleration/deceleration rates.
     */
    class TurnMotion final : public Motion
    {
    public:
        TurnMotion(MotionContext ctx, TurnConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        /**
         * Begin turn from current state without resetting odometry.
         *
         * Used by smooth_path() to carry angular velocity across segment boundaries.
         *
         * @param heading_offset_rad  Current accumulated heading (subtracted from reads)
         * @param initial_angular_velocity  Current angular velocity to seed the profile with
         */
        void startWarm(double heading_offset_rad, double initial_angular_velocity);

        /** When true, complete() sets finished_ without calling hardStop(). */
        void setSuppressHardStopOnComplete(bool suppress) { suppress_hard_stop_ = suppress; }

        /** Check if target angle is reached, ignoring velocity settling. */
        [[nodiscard]] bool hasReachedAngle() const;

        /** Current filtered angular velocity (rad/s). */
        [[nodiscard]] double getFilteredVelocity() const { return filtered_velocity_; }

    private:
        void complete();

        TurnConfig cfg_{};
        double max_velocity_{0.0};  // computed from speed_scale * AxisConstraints
        bool finished_{false};
        bool suppress_hard_stop_{false};
        double heading_offset_rad_{0.0};  // subtracted from accumulated heading in warm-start mode

        // Profiled PID controller (replaces manual PID)
        ProfiledPIDController profiled_pid_;

        // Velocity tracking for settling detection
        double prev_heading_{0.0};       // last wrapped heading from odometry (for delta computation)
        double accumulated_heading_{0.0}; // unwrapped heading (handles wraparound at ±π)
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};

        // Diagnostic counter
        int cycle_{0};
    };
}
