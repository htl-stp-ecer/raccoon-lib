#pragma once

#include "foundation/pid.hpp"

namespace libstp::motion
{
    /**
     * Per-axis motion profile constraints (max velocity, acceleration, deceleration).
     * Measured from actual robot response via characterize_drive().
     * 0.0 = not measured / use fallback.
     */
    struct AxisConstraints
    {
        double max_velocity{0.0};
        double acceleration{0.0};
        double deceleration{0.0};

        AxisConstraints() = default;

        AxisConstraints(double max_velocity, double acceleration, double deceleration)
            : max_velocity(max_velocity), acceleration(acceleration), deceleration(deceleration)
        {}
    };

    /**
     * Unified PID and saturation configuration for all motion primitives.
     *
     * This configuration is intended to be robot-wide and speed-independent.
     * The axis constraints should come from measured robot behavior rather than
     * hand-picked guesses.
     */
    struct UnifiedMotionPidConfig
    {
        // Distance/Position PID (used for forward/backward distance, lateral distance)
        foundation::PidConfig distance{2.0, 0.0, 0.5, 10.0, 0.01, 0.3};

        // Heading/Angular PID (used for maintaining heading in drive/strafe, and for turning)
        foundation::PidConfig heading{2.0, 0.0, 0.3, 10.0, 0.01, 0.3};

        // Profiled PID velocity feedforward gain.
        // 1.0 = full feedforward from profile velocity (recommended for velocity-commanding systems).
        // 0.0 = pure PID on position error (WPILib-style, needs larger kP).
        double velocity_ff{1.0};

        // Saturation handling (speed-independent derating factors)
        double saturation_derating_factor{0.9};
        double saturation_min_scale{0.2};
        double saturation_recovery_rate{0.03};

        // Heading-specific saturation handling
        double heading_saturation_derating_factor{0.85};
        double heading_min_scale{0.25};
        double heading_recovery_rate{0.05};

        // Hysteresis parameters to prevent oscillation
        int saturation_hold_cycles{5};
        double saturation_recovery_threshold{0.95};

        // Tolerances
        double distance_tolerance_m{0.01};
        double angle_tolerance_rad{0.035};

        // Saturation heading thresholds
        double heading_saturation_error_rad{0.01};
        double heading_recovery_error_rad{0.005};

        // Per-axis motion profile constraints, typically populated from
        // characterization data such as characterize_drive().
        AxisConstraints linear{};
        AxisConstraints lateral{};
        AxisConstraints angular{};
    };

    /**
     * Default unified PID configuration.
     */
    inline UnifiedMotionPidConfig defaultUnifiedMotionPidConfig()
    {
        return UnifiedMotionPidConfig{};
    }
}
