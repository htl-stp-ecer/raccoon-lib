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
        double distance_kd{0.5};            // Velocity damping for lag compensation in LinearMotion

        // Heading/Angular PID gains (used for maintaining heading in drive/strafe, and for turning)
        double heading_kp{2.0};
        double heading_ki{0.0};
        double heading_kd{0.3};

        // Lateral drift correction PID gains (used for correcting lateral drift during drive)
        double lateral_kp{2.0};
        double lateral_ki{0.0};
        double lateral_kd{0.0};

        // Advanced PID parameters (shared across all controllers)
        double integral_max{10.0};              // Anti-windup limit for all PIDs
        double integral_deadband{0.01};         // Don't integrate within this error
        double derivative_lpf_alpha{0.3};       // Low-pass filter coefficient for derivative term
        double output_min{-10.0};               // Minimum PID output
        double output_max{10.0};                // Maximum PID output

        // Saturation handling (speed-independent derating factors)
        double saturation_derating_factor{0.9};     // Multiply speed scale on saturation
        double saturation_min_scale{0.2};           // Never reduce speed scale below this
        double saturation_recovery_rate{0.03};      // How quickly speed scale recovers per cycle

        // Heading-specific saturation handling
        double heading_saturation_derating_factor{0.85};  // Multiply heading scale when still saturated
        double heading_min_scale{0.25};                   // Minimum heading scale
        double heading_recovery_rate{0.05};               // Recovery rate for heading scale

        // Hysteresis parameters to prevent oscillation
        int saturation_hold_cycles{5};              // Cycles to hold before starting recovery
        double saturation_recovery_threshold{0.95}; // Only recover when scale is below this

        // Tolerances
        double distance_tolerance_m{0.01};       // Position completion tolerance (meters)
        double angle_tolerance_rad{0.035};       // Angular completion tolerance (radians, ~2 degrees)

        // Lateral drift handling (for differential drive)
        double lateral_heading_bias_gain{0.5};          // How much to bias heading based on lateral error
        double lateral_reorient_threshold_m{0.15};      // Stop and reorient if lateral error exceeds this
        double heading_saturation_error_rad{0.01};      // Only derate when heading error exceeds this
        double heading_recovery_error_rad{0.005};       // Allow recovery once heading error is below this

        // Minimum speeds (to prevent stalling)
        double min_speed_mps{0.05};              // Minimum translational speed

        // Reorientation behavior (differential drive)
        double reorientation_speed_factor{0.3};  // Reduce forward speed to this fraction during reorientation

        // Response lag compensation (Smith predictor) - used by TurnMotion
        double response_lag_s{0.3};              // Actuator response lag (seconds)

        // Braking-distance motion profile (LinearMotion)
        double decel_mps2{0.10};                 // Conservative deceleration rate for braking calculation (m/s²).
                                                 // v_brake = sqrt(2 * decel * d_remaining) determines when to brake.
                                                 // Lower = earlier braking (safer), higher = later braking (faster).
                                                 // Combined with distance_kd velocity damping for lag compensation.
        double rest_horizon_s{0.7};              // Prediction horizon when starting from rest (seconds).
                                                 // Should match the from-rest latency (~600-700ms).
        double horizon_blend_speed_mps{0.1};     // Speed at which horizon transitions from rest to move (m/s).
                                                 // Below this: use rest_horizon_s. Above: use response_lag_s.

        UnifiedMotionPidConfig() = default;

        UnifiedMotionPidConfig(
            double distance_kp, double distance_ki, double distance_kd,
            double heading_kp, double heading_ki, double heading_kd,
            double lateral_kp, double lateral_ki, double lateral_kd,
            double integral_max, double integral_deadband, double derivative_lpf_alpha,
            double output_min, double output_max,
            double saturation_derating_factor, double saturation_min_scale, double saturation_recovery_rate,
            double heading_saturation_derating_factor, double heading_min_scale, double heading_recovery_rate,
            int saturation_hold_cycles, double saturation_recovery_threshold,
            double distance_tolerance_m, double angle_tolerance_rad,
            double lateral_heading_bias_gain, double lateral_reorient_threshold_m,
            double heading_saturation_error_rad, double heading_recovery_error_rad,
            double min_speed_mps,
            double reorientation_speed_factor,
            double response_lag_s,
            double decel_mps2, double rest_horizon_s, double horizon_blend_speed_mps
        ) : distance_kp(distance_kp), distance_ki(distance_ki), distance_kd(distance_kd),
            heading_kp(heading_kp), heading_ki(heading_ki), heading_kd(heading_kd),
            lateral_kp(lateral_kp), lateral_ki(lateral_ki), lateral_kd(lateral_kd),
            integral_max(integral_max), integral_deadband(integral_deadband),
            derivative_lpf_alpha(derivative_lpf_alpha),
            output_min(output_min), output_max(output_max),
            saturation_derating_factor(saturation_derating_factor),
            saturation_min_scale(saturation_min_scale),
            saturation_recovery_rate(saturation_recovery_rate),
            heading_saturation_derating_factor(heading_saturation_derating_factor),
            heading_min_scale(heading_min_scale), heading_recovery_rate(heading_recovery_rate),
            saturation_hold_cycles(saturation_hold_cycles),
            saturation_recovery_threshold(saturation_recovery_threshold),
            distance_tolerance_m(distance_tolerance_m), angle_tolerance_rad(angle_tolerance_rad),
            lateral_heading_bias_gain(lateral_heading_bias_gain),
            lateral_reorient_threshold_m(lateral_reorient_threshold_m),
            heading_saturation_error_rad(heading_saturation_error_rad),
            heading_recovery_error_rad(heading_recovery_error_rad),
            min_speed_mps(min_speed_mps),
            reorientation_speed_factor(reorientation_speed_factor),
            response_lag_s(response_lag_s),
            decel_mps2(decel_mps2), rest_horizon_s(rest_horizon_s),
            horizon_blend_speed_mps(horizon_blend_speed_mps)
        {}
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
