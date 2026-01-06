#pragma once

#include <Eigen/Geometry>
#include <memory>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/trapezoidal_profile.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    class MotionPidController;

    struct DriveStraightConfig
    {
        double distance_m{0.0};
        double max_speed_mps{0.0};
        // Note: PID gains, tolerances, and saturation parameters are now in UnifiedMotionPidConfig (accessible via MotionContext)
    };

    class DriveStraightMotion final : public Motion
    {
    public:
        /// @deprecated Use the type-safe overload with Meters and MetersPerSecond instead
        [[deprecated("Use DriveStraightMotion(ctx, Meters, MetersPerSecond) for type safety")]]
        DriveStraightMotion(MotionContext ctx, double distance_cm, double max_speed_mps);

        /// Type-safe constructor using strongly-typed units
        DriveStraightMotion(MotionContext ctx, foundation::Meters distance, foundation::MetersPerSecond max_speed);

        DriveStraightMotion(MotionContext ctx, DriveStraightConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;
    private:
        void complete();

        DriveStraightConfig cfg_{};
        std::unique_ptr<MotionPidController> distance_pid_;  // PID controller for distance
        std::unique_ptr<MotionPidController> heading_pid_;   // PID controller for heading
        std::unique_ptr<MotionPidController> lateral_pid_;   // PID controller for lateral drift
        std::unique_ptr<TrapezoidalProfile> profile_;        // Trapezoidal profile for smooth setpoint
        double initial_heading_rad_{0.0};  // Heading at start (target heading to maintain)
        double elapsed_time_{0.0};         // Time elapsed since start
        bool finished_{false};
        double speed_scale_{1.0};    // Current translational speed scaling factor due to saturation
        double heading_scale_{1.0};  // Current heading command scaling factor
        bool reorienting_{false};  // For differential: currently in reorientation mode
        int unsaturated_cycles_{0};  // Hysteresis counter for saturation recovery
    };
}
