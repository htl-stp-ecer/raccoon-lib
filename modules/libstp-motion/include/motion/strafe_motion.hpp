#pragma once

#include "motion/motion.hpp"
#include <memory>

namespace libstp::motion
{
    class MotionPidController;

    struct StrafeConfig
    {
        double target_distance_m{0.0};        // Target strafe distance (positive = left, negative = right)
        double max_speed_mps{0.3};            // Maximum strafing speed (m/s)
        double distance_tolerance_m{0.02};    // Completion tolerance
        double distance_kp{2.0};              // Proportional gain for distance control
        double distance_ki{0.0};              // Integral gain for distance control
        double distance_kd{0.0};              // Derivative gain for distance control
        double heading_kp{3.0};               // Proportional gain for heading correction
        double heading_ki{0.0};               // Integral gain for heading correction
        double heading_kd{0.0};               // Derivative gain for heading correction
        double min_speed_mps{0.05};           // Minimum speed to prevent stalling
        double saturation_derating_factor{0.85};  // Multiply speed by this on each saturation hit
        double saturation_min_scale{0.2};         // Never reduce speed scale below this
        double saturation_recovery_rate{0.05};    // How quickly speed scale recovers per cycle
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
        double initial_heading_rad_{0.0};  // Initial heading to maintain
        double target_distance_m_{0.0};     // Target lateral distance to travel
        bool finished_{false};
        double speed_scale_{1.0};  // Current speed scaling factor due to saturation
    };
}
