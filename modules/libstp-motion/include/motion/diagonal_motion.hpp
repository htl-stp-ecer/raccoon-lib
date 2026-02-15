#pragma once

#include <memory>
#include <vector>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/profiled_pid_controller.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    struct DiagonalMotionConfig
    {
        double angle_rad{0.0};               // Travel angle: 0 = forward, pi/2 = right, -pi/2 = left
        double distance_m{0.0};              // Distance along the travel direction
        double max_speed_mps{0.0};           // Max speed along travel direction
        double max_acceleration_mps2{0.0};   // m/s² (0 = use default from pid_config)
        double max_deceleration_mps2{0.0};   // m/s² (0 = use acceleration, or default)
    };

    struct DiagonalMotionTelemetry
    {
        double time_s{0.0};             // elapsed since start
        double dt{0.0};                 // this cycle's dt
        // Position
        double target_m{0.0};           // target distance along travel direction
        double position_m{0.0};         // actual position along travel direction
        double predicted_m{0.0};        // profile setpoint position
        double cross_track_m{0.0};      // cross-track drift (perpendicular to travel)
        // Errors
        double distance_error_m{0.0};   // target - actual along travel
        double actual_error_m{0.0};     // target - actual
        double yaw_error_rad{0.0};      // heading error
        // Velocity
        double filtered_velocity_mps{0.0};
        // Commands (after clamp + scaling)
        double cmd_vx_mps{0.0};         // final commanded vx
        double cmd_vy_mps{0.0};         // final commanded vy
        double cmd_wz_radps{0.0};       // final commanded omega
        // Raw PID outputs (before clamp/scaling)
        double pid_primary_raw{0.0};    // profiled PID raw output
        double pid_cross_raw{0.0};      // cross-track PID raw output
        double pid_heading_raw{0.0};    // heading PID raw output
        // Profile state
        double setpoint_position_m{0.0};
        double setpoint_velocity_mps{0.0};
        // State
        double heading_rad{0.0};        // current heading
        double speed_scale{1.0};
        double heading_scale{1.0};
        bool saturated{false};
    };

    class DiagonalMotion final : public Motion
    {
    public:
        DiagonalMotion(MotionContext ctx, DiagonalMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        [[nodiscard]] const std::vector<DiagonalMotionTelemetry>& getTelemetry() const { return telemetry_; }

    private:
        void complete();

        DiagonalMotionConfig cfg_{};
        double cos_angle_{1.0};  // precomputed cos(angle_rad)
        double sin_angle_{0.0};  // precomputed sin(angle_rad)
        std::unique_ptr<foundation::PidController> heading_pid_;
        ProfiledPIDController profiled_pid_;
        double initial_heading_rad_{0.0};
        bool finished_{false};
        double speed_scale_{1.0};
        double heading_scale_{1.0};
        int unsaturated_cycles_{0};

        // Velocity tracking for settling detection
        double prev_primary_position_{0.0};
        double filtered_velocity_{0.0};
        static constexpr double kVelocityFilterAlpha{0.3};
        static constexpr double kSettlingVelocity{0.02}; // m/s - must be nearly stopped to declare done

        // Telemetry
        double elapsed_time_{0.0};
        std::vector<DiagonalMotionTelemetry> telemetry_;
    };
}
