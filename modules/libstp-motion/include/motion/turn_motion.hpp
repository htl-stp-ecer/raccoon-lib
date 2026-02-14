#pragma once

#include "motion/motion.hpp"
#include "motion/profiled_pid_controller.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    struct TurnConfig
    {
        double target_angle_rad{0.0};        // Target turn angle (positive = CCW, negative = CW)
        double max_angular_rate{2.0};        // Maximum turning speed (rad/s)
        double max_angular_acceleration{2.0}; // rad/s² (acceleration rate for profile)
        double max_angular_deceleration{0.0}; // rad/s² (deceleration rate, 0 = use acceleration)
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

        // Profiled PID controller (replaces manual PID)
        ProfiledPIDController profiled_pid_;

        // Velocity tracking for settling detection
        double prev_heading_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};
    };
}
