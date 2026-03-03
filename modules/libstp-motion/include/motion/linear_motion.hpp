#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "motion/motion.hpp"
#include "motion/motion_pid.hpp"
#include "motion/profiled_pid_controller.hpp"
#include "foundation/types.hpp"

namespace libstp::motion
{
    /** Selects which body-frame axis `LinearMotion` should move along. */
    enum class LinearAxis { Forward, Lateral };

    /**
     * Configuration for a straight-line motion along one chassis axis.
     *
     * `distance_m` follows the robot body frame:
     * - forward axis: positive = forward, negative = backward
     * - lateral axis: positive = right, negative = left
     */
    struct LinearMotionConfig
    {
        LinearAxis axis{LinearAxis::Forward};
        double distance_m{0.0};
        double speed_scale{1.0};             // 0-1 fraction of AxisConstraints.max_velocity
    };

    /** Per-cycle diagnostics captured while a `LinearMotion` instance runs. */
    struct LinearMotionTelemetry
    {
        double time_s{0.0};             // elapsed since start
        double dt{0.0};                 // this cycle's dt
        // Position
        double target_m{0.0};           // target distance
        double position_m{0.0};         // actual primary axis position
        double predicted_m{0.0};        // profile setpoint position
        double cross_track_m{0.0};      // cross-track drift
        // Errors
        double distance_error_m{0.0};   // target - actual
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

    /**
     * Closed-loop straight-line controller for forward/backward or lateral motion.
     *
     * The controller follows a trapezoidal profile on the selected primary axis
     * while correcting heading error and cross-track drift.
     */
    class LinearMotion final : public Motion
    {
    public:
        LinearMotion(MotionContext ctx, LinearMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        /** Telemetry samples appended on each update cycle. */
        [[nodiscard]] const std::vector<LinearMotionTelemetry>& getTelemetry() const { return telemetry_; }

        /// Override the internal heading PID with an external omega value.
        /// Set each cycle before calling update(). Persists until cleared.
        void setOmegaOverride(double omega) { omega_override_ = omega; }
        void clearOmegaOverride() { omega_override_.reset(); }

    private:
        void complete();

        LinearMotionConfig cfg_{};
        double max_velocity_{0.0};  // computed from speed_scale * AxisConstraints
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

        // External omega override (replaces heading PID when set)
        std::optional<double> omega_override_{};

        // Telemetry
        double elapsed_time_{0.0};
        std::vector<LinearMotionTelemetry> telemetry_;
    };
}
