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

    private:
        void complete();

        TurnConfig cfg_{};
        double max_velocity_{0.0};  // computed from speed_scale * AxisConstraints
        bool finished_{false};

        // Profiled PID controller (replaces manual PID)
        ProfiledPIDController profiled_pid_;

        // Velocity tracking for settling detection
        double prev_heading_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};

        // Diagnostic counter
        int cycle_{0};
    };
}
