#pragma once

#include "motion/motion.hpp"

namespace libstp::motion
{
    struct TurnConfig
    {
        double target_angle_rad{0.0};        // Target turn angle (positive = CCW, negative = CW)
        double max_angular_rate{1.0};        // Maximum turning speed (rad/s)
        double angle_tolerance_rad{0.02};    // Completion tolerance (~1.15 degrees)
        double angle_kp{3.0};                // Proportional gain for angular control
        double min_angular_rate{0.1};        // Minimum turning speed to prevent stalling
        double saturation_derating_factor{0.85};  // Multiply angular speed by this on each saturation hit
        double saturation_min_scale{0.2};         // Never reduce angular scale below this
        double saturation_recovery_rate{0.05};    // How quickly angular scale recovers per cycle
    };

    class TurnMotion final : public Motion
    {
    public:
        TurnMotion(MotionContext ctx, double angle_deg, double max_angular_rate_rad_per_sec);
        TurnMotion(MotionContext ctx, TurnConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

    private:
        void complete();

        TurnConfig cfg_{};
        double target_heading_rad_{0.0};  // Target heading to reach
        bool finished_{false};
        double angular_scale_{1.0};  // Current angular speed scaling factor due to saturation
    };
}
