#pragma once

namespace libstp::motion
{
    /**
     * Unified PID configuration for all motion primitives.
     * This configuration is speed-independent and can be set once in config.yml.
     */
    struct UnifiedMotionPidConfig
    {
        // Distance/Position PID gains (used for forward/backward distance in drive, lateral distance in strafe)
        double distance_kp{2.0};
        double distance_ki{0.0};
        double distance_kd{0.0};

        // Heading/Angular PID gains (used for maintaining heading in drive/strafe, and for turning)
        double heading_kp{3.0};
        double heading_ki{0.0};
        double heading_kd{0.0};

        // Lateral drift correction PID gains (used for correcting lateral drift during drive)
        double lateral_kp{2.0};
        double lateral_ki{0.0};
        double lateral_kd{0.0};

        // Trapezoidal profile parameters (for smooth motion with ramped setpoints)
        double max_linear_acceleration{1.0};    // Maximum linear acceleration (m/s²) for drive/strafe
        double max_angular_acceleration{3.0};   // Maximum angular acceleration (rad/s²) for turns

        // Advanced PID parameters (shared across all controllers)
        double integral_max{10.0};              // Anti-windup limit for all PIDs
        double integral_deadband{0.01};         // Don't integrate within this error
        double derivative_lpf_alpha{0.1};       // Low-pass filter coefficient for derivative term
        double output_min{-10.0};               // Minimum PID output
        double output_max{10.0};                // Maximum PID output

        // Saturation handling (speed-independent derating factors)
        double saturation_derating_factor{0.85};    // Multiply speed scale on saturation
        double saturation_min_scale{0.1};           // Never reduce speed scale below this
        double saturation_recovery_rate{0.02};      // How quickly speed scale recovers per cycle

        // Heading-specific saturation handling
        double heading_saturation_derating_factor{0.85};  // Multiply heading scale when still saturated
        double heading_min_scale{0.25};                   // Minimum heading scale
        double heading_recovery_rate{0.05};               // Recovery rate for heading scale

        // Hysteresis parameters to prevent oscillation
        int saturation_hold_cycles{5};              // Cycles to hold before starting recovery
        double saturation_recovery_threshold{0.95}; // Only recover when scale is below this

        // Tolerances
        double distance_tolerance_m{0.01};       // Position completion tolerance (meters)
        double angle_tolerance_rad{0.02};        // Angular completion tolerance (radians, ~1.15 degrees)

        // Rate limits (these are speed-related but define maximum rates, not gains)
        double max_heading_rate{3.0};            // Maximum heading correction rate (rad/s)
        double min_angular_rate{0.1};            // Minimum turning speed to prevent stalling

        // Lateral drift handling (for differential drive)
        double lateral_heading_bias_gain{0.5};          // How much to bias heading based on lateral error
        double lateral_reorient_threshold_m{0.15};      // Stop and reorient if lateral error exceeds this
        double heading_saturation_error_rad{0.01};      // Only derate when heading error exceeds this
        double heading_recovery_error_rad{0.005};       // Allow recovery once heading error is below this

        // Minimum speeds (to prevent stalling)
        double min_speed_mps{0.05};              // Minimum translational speed

        // Reorientation behavior (differential drive)
        double reorientation_speed_factor{0.3};  // Reduce forward speed to this fraction during reorientation
    };

    /**
     * Default unified PID configuration.
     * These values are tuned to work well across different robot types and speeds.
     */
    inline UnifiedMotionPidConfig defaultUnifiedMotionPidConfig()
    {
        return UnifiedMotionPidConfig{};
    }
}
