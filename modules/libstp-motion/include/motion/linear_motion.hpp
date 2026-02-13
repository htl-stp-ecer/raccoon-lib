#pragma once

#include <memory>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/trapezoidal_profile.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    class MotionPidController;

    enum class LinearAxis { Forward, Lateral };

    struct LinearMotionConfig
    {
        LinearAxis axis{LinearAxis::Forward};
        double distance_m{0.0};
        double max_speed_mps{0.0};
        double max_acceleration_mps2{1.0};
    };

    class LinearMotion final : public Motion
    {
    public:
        LinearMotion(MotionContext ctx, LinearMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

    private:
        void complete();

        LinearMotionConfig cfg_{};
        std::unique_ptr<MotionPidController> distance_pid_;   // PID controller for primary axis distance
        std::unique_ptr<MotionPidController> heading_pid_;    // PID controller for heading
        std::unique_ptr<MotionPidController> cross_track_pid_;  // PID controller for cross-track drift
        std::unique_ptr<TrapezoidalProfile> profile_;         // Trapezoidal profile for smooth setpoint
        double initial_heading_rad_{0.0};
        double elapsed_time_{0.0};
        bool finished_{false};
        double speed_scale_{1.0};
        double heading_scale_{1.0};
        bool reorienting_{false};
        int unsaturated_cycles_{0};
    };
}
