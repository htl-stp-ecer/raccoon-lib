#pragma once

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "foundation/types.hpp"
#include <memory>

namespace libstp::motion
{
    class MotionPidController;

    struct TurnConfig
    {
        double target_angle_rad{0.0};        // Target turn angle (positive = CCW, negative = CW)
        double max_angular_rate{1.0};        // Maximum turning speed (rad/s)
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
        std::unique_ptr<MotionPidController> pid_;
        double target_heading_rad_{0.0};
        bool finished_{false};

        // Velocity tracking for settling check
        double prev_heading_{0.0};
    };
}
