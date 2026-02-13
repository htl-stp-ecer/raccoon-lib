#pragma once

#include "motion/motion.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    struct TurnConfig
    {
        double target_angle_rad{0.0};        // Target turn angle (positive = CCW, negative = CW)
        double max_angular_rate{2.0};        // Maximum turning speed (rad/s)
        double max_angular_acceleration{2.0}; // rad/s² (reserved for future use)
        double kS{0.0};                      // Static friction compensation (rad/s)
    };

    /**
     * PID turn controller with output-clamped velocity command.
     *
     * Uses a simple PID on heading error.  The proportional term saturated by
     * max_angular_rate naturally produces a trapezoidal-like velocity profile:
     * full speed while far from goal, proportional deceleration near it.
     * The derivative term damps momentum to prevent overshoot.
     */
    class TurnMotion final : public Motion
    {
    public:
        /// @deprecated Use the type-safe overload with Radians and RadiansPerSecond instead
        [[deprecated("Use TurnMotion(ctx, Radians, RadiansPerSecond) for type safety")]]
        TurnMotion(MotionContext ctx, double angle_deg, double max_angular_rate_rad_per_sec);

        /// Type-safe constructor using strongly-typed units
        TurnMotion(MotionContext ctx, foundation::Radians angle, foundation::RadiansPerSecond max_angular_rate);

        TurnMotion(MotionContext ctx, TurnConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

    private:
        void complete();

        TurnConfig cfg_{};
        bool finished_{false};

        // PID gains (from pid_config heading gains)
        double kP_{0.0};
        double kI_{0.0};
        double kD_{0.0};

        // PID state
        double prev_error_{0.0};
        double total_error_{0.0};
        double filtered_derivative_{0.0};
        int last_error_sign_{0};

        // Velocity tracking for settling detection
        double prev_heading_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};
    };
}
