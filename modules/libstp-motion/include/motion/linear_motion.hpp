#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

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

        /// Absolute world-frame heading (radians) the controller holds during
        /// the motion. Required — the path executor sets this per segment.
        /// Heading PID error is computed against `getAbsoluteHeading()`.
        double target_heading_rad{0.0};

        /// True (default) when `distance_m` is a real positional target that
        /// drives motion completion. Set to false when callers use the
        /// controller in "until" mode — `distance_m` is then a sentinel
        /// upper bound and the motion terminates via an external condition.
        ///
        /// SpeedMode (BEMF disabled) makes distance-based termination
        /// unreliable; `start()` raises `std::logic_error` if this flag is
        /// true while SpeedMode is active.
        bool has_distance_target{true};
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

        /**
         * Begin motion from current state without resetting odometry.
         *
         * Used by smooth_path() to carry velocity across segment boundaries.
         * The profiled PID starts at position 0 with the given initial velocity,
         * and odometry reads are offset so the segment measures from its start point.
         *
         * @param position_offset_m  Current primary-axis odometry position (subtracted from reads)
         * @param initial_velocity_mps  Current velocity to seed the profile with
         */
        void startWarm(double position_offset_m, double initial_velocity_mps);

        /** When true, complete() sets finished_ without calling hardStop(). */
        void setSuppressHardStopOnComplete(bool suppress) { suppress_hard_stop_ = suppress; }

        /** Check if target distance is reached, ignoring velocity settling. */
        [[nodiscard]] bool hasReachedDistance() const;

        /** Current filtered velocity along the primary axis (m/s). */
        [[nodiscard]] double getFilteredVelocity() const { return filtered_velocity_; }

        /** Telemetry samples appended on each update cycle. */
        [[nodiscard]] const std::vector<LinearMotionTelemetry>& getTelemetry() const { return telemetry_; }

        /// Override the internal heading PID with an external omega value.
        /// Set each cycle before calling update(). Persists until cleared.
        void setOmegaOverride(double omega) { omega_override_ = omega; }
        void clearOmegaOverride() { omega_override_.reset(); }

    private:
        void complete();

        /// Snapshot the current pose as the body-frame origin for this motion.
        /// Replaces the historical `odometry().reset()` call in start()/startWarm().
        void captureInitialPose();

        /// Project the displacement since start into a body frame aligned with
        /// the *target* heading (cfg_.target_heading_rad), not the heading
        /// captured at start. This keeps the measured forward distance aligned
        /// with the direction the heading PID actually drives toward, so an
        /// absolute-heading motion that begins with a heading error does not
        /// overshoot. Returns (forward, lateral) signed distances; lateral is
        /// positive to the right of the target heading.
        [[nodiscard]] std::pair<double, double> projectBodyFrame() const;

        LinearMotionConfig cfg_{};
        double max_velocity_{0.0};  // computed from speed_scale * AxisConstraints
        std::unique_ptr<foundation::PidController> heading_pid_;
        ProfiledPIDController profiled_pid_;
        Eigen::Vector2d initial_position_m_{Eigen::Vector2d::Zero()};
        double initial_heading_rad_{0.0};
        bool finished_{false};
        bool suppress_hard_stop_{false};
        double position_offset_m_{0.0};  // subtracted from odometry in warm-start mode
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

        // Slew-limit state for the internal heading-hold omega output
        double prev_omega_cmd_{0.0};

        // Telemetry
        double elapsed_time_{0.0};
        // At 100 Hz update rate, 30 s of motion = 3000 samples × ~80 bytes
        // ≈ 240 KB. Capping at this size prevents long-running motions
        // (timeouts, condition-driven drives) from leaking memory across an
        // autonomous period. Older samples are dropped from the front; the
        // most recent window is what's interesting for tuning anyway.
        static constexpr size_t kMaxTelemetrySamples{3000};
        std::vector<LinearMotionTelemetry> telemetry_;
    };
}
