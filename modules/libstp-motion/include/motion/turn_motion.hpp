#pragma once

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/trapezoidal_profile.hpp"
#include "foundation/types.hpp"
#include <memory>

namespace libstp::motion
{
    class MotionPidController;

    struct TurnConfig
    {
        double target_angle_rad{0.0};        // Target turn angle (positive = CCW, negative = CW)
        double max_angular_rate{1.0};        // Maximum turning speed (rad/s)
        // Note: PID gains, tolerances, and saturation parameters are now in UnifiedMotionPidConfig (accessible via MotionContext)
    };

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
        std::unique_ptr<MotionPidController> angle_pid_;  // PID controller for angle control
        std::unique_ptr<TrapezoidalProfile> profile_;     // Trapezoidal profile for smooth setpoint
        double target_heading_rad_{0.0};  // Final target heading to reach
        double elapsed_time_{0.0};        // Time elapsed since start
        bool finished_{false};
        double angular_scale_{1.0};  // Current angular speed scaling factor due to saturation
        int unsaturated_cycles_{0};  // Hysteresis counter for saturation recovery
    };
}
