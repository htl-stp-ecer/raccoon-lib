#pragma once

#include "motion/motion.hpp"
#include "motion/trapezoidal_profile.hpp"
#include <memory>

namespace libstp::motion
{
    class MotionPidController;

    struct StrafeConfig
    {
        double target_distance_m{0.0};        // Target strafe distance (positive = left, negative = right)
        double max_speed_mps{0.3};            // Maximum strafing speed (m/s)
        // Note: PID gains, tolerances, and saturation parameters are now in UnifiedMotionPidConfig (accessible via MotionContext)
    };

    class StrafeMotion final : public Motion
    {
    public:
        StrafeMotion(MotionContext ctx, double distance_m, double max_speed_mps);
        StrafeMotion(MotionContext ctx, StrafeConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

    private:
        void complete();

        StrafeConfig cfg_{};
        std::unique_ptr<MotionPidController> lateral_pid_;   // PID controller for lateral distance
        std::unique_ptr<MotionPidController> heading_pid_;   // PID controller for heading
        std::unique_ptr<TrapezoidalProfile> profile_;        // Trapezoidal profile for smooth setpoint
        double initial_heading_rad_{0.0};  // Initial heading to maintain
        double target_distance_m_{0.0};     // Target lateral distance to travel
        double elapsed_time_{0.0};          // Time elapsed since start
        bool finished_{false};
        double speed_scale_{1.0};  // Current speed scaling factor due to saturation
    };
}
