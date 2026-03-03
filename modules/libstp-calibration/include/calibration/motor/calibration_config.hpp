//
// Created by tobias on 11/28/25.
//

#pragma once
#include <string>
#include <vector>

namespace libstp::calibration
{
    /**
     * Configuration for the motor calibration pipeline.
     *
     * Most values are expressed in seconds, meters, percent motor command,
     * or physical units derived from encoder velocity. Update the contributor
     * docs whenever a new field changes the public workflow.
     */
    struct CalibrationConfig
    {
        // Step response tuning parameters
        double step_response_amplitude{30.0};  // Command percentage for step test
        double step_response_duration{3.0};    // Duration of step test in seconds

        // Relay feedback tuning parameters
        double relay_amplitude{50.0};          // Command percentage for relay test
        double max_relay_duration{10.0};       // Maximum duration for relay test
        int min_oscillations{3};               // Minimum oscillations to measure
        bool use_relay_feedback{false};        // Use relay feedback instead of step response

        // Feedforward calibration parameters
        double static_friction_increment{0.5}; // Command increment for static friction search (%)
        double static_friction_max{40.0};      // Maximum command to try for static friction
        double min_velocity_threshold{1.0};    // Minimum velocity (rad/s) to consider movement (above encoder noise)
        std::vector<double> velocity_test_commands{10.0, 15.0, 20.0, 25.0}; // Test speeds (%) - lower to avoid saturation
        double velocity_test_duration{1.5};    // Duration per velocity test
        double velocity_settling_time{0.5};    // Time to wait for settling
        int acceleration_test_count{3};        // Number of acceleration tests to average
        double acceleration_test_amplitude{30.0}; // Command for acceleration tests - lower to stay in linear range

        // Safety parameters
        double max_test_distance_m{0.6};       // Emergency stop distance
        double max_single_test_duration{60.0}; // Timeout per individual test
        double max_calibration_duration{120.0}; // Total calibration timeout
        int max_retries{3};                    // Retry attempts on failure

        // Validation parameters
        double validation_duration{2.0};       // Duration of validation test
        double validation_max_error{0.2};      // Maximum acceptable tracking error (20%)
        std::vector<double> validation_test_commands{10.0, 15.0, 20.0}; // Validation command set (%)

        // Validation data export (optional)
        bool export_validation_profiles{false}; // Export validation command vs measured data
        std::string validation_output_dir{"logs/motor_validation"}; // CSV output directory

        // Parameter range validation
        bool validate_parameter_ranges{false}; // Enforce ranges below when true

        // Value range validation (normalized units 0-1 for feedforward)
        struct ValidationRanges {
            double kS_min{0.0};      // Static friction in normalized units [0-1]
            double kS_max{0.20};     // Static friction in normalized units [0-1] (20% max)
            double kV_min{0.01};     // Velocity constant in normalized per (rad/s) - lowered for your motors
            double kV_max{1.0};      // Velocity constant in normalized per (rad/s)
            double kA_min{0.0001};   // Acceleration constant in normalized per (rad/s²)
            double kA_max{2.0};      // Acceleration constant in normalized per (rad/s²) - raised to allow ~1% in percent units
            double kp_min{0.1};
            double kp_max{10.0};
            double ki_min{0.0};
            double ki_max{10.0};     // Motor velocity control typically needs moderate ki
            double kd_min{0.0};
            double kd_max{2.0};
        } ranges;
    };
}
