#pragma once

#include <memory>
#include <vector>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
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
    };

    struct LinearMotionTelemetry
    {
        double time_s{0.0};             // elapsed since start
        double dt{0.0};                 // this cycle's dt
        // Position
        double target_m{0.0};           // target distance
        double position_m{0.0};         // actual primary axis position
        double predicted_m{0.0};        // Smith predictor output
        double cross_track_m{0.0};      // cross-track drift
        // Errors
        double distance_error_m{0.0};   // target - predicted
        double actual_error_m{0.0};     // target - actual
        double yaw_error_rad{0.0};      // heading error
        // Velocity
        double filtered_velocity_mps{0.0};
        // Commands (after clamp + scaling)
        double cmd_vx_mps{0.0};         // final commanded vx
        double cmd_vy_mps{0.0};         // final commanded vy
        double cmd_wz_radps{0.0};       // final commanded omega
        // Raw PID outputs (before clamp/scaling)
        double pid_primary_raw{0.0};    // distance PID raw output
        double pid_cross_raw{0.0};      // cross-track PID raw output
        double pid_heading_raw{0.0};    // heading PID raw output
        // State
        double heading_rad{0.0};        // current heading
        double speed_scale{1.0};
        double heading_scale{1.0};
        bool saturated{false};
    };

    class LinearMotion final : public Motion
    {
    public:
        LinearMotion(MotionContext ctx, LinearMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        [[nodiscard]] const std::vector<LinearMotionTelemetry>& getTelemetry() const { return telemetry_; }

    private:
        void complete();

        LinearMotionConfig cfg_{};
        std::unique_ptr<MotionPidController> distance_pid_;   // PID controller for primary axis distance
        std::unique_ptr<MotionPidController> heading_pid_;    // PID controller for heading
        std::unique_ptr<MotionPidController> cross_track_pid_;  // PID controller for cross-track drift
        double initial_heading_rad_{0.0};
        bool finished_{false};
        double speed_scale_{1.0};
        double heading_scale_{1.0};
        bool reorienting_{false};
        int unsaturated_cycles_{0};

        // Velocity tracking for settling detection
        double prev_primary_position_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};
        static constexpr double kSettlingVelocity{0.02}; // m/s - must be nearly stopped to declare done

        // Telemetry
        double elapsed_time_{0.0};
        std::vector<LinearMotionTelemetry> telemetry_;
    };
}
