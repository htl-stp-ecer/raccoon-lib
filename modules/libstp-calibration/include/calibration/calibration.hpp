//
// Created by Claude Code for automatic motor calibration
//

#pragma once

#include "foundation/motor.hpp"
#include "hal/Motor.hpp"
#include <vector>
#include <string>
#include <memory>

namespace libstp::drive
{
    // Forward declaration
    class MotorAdapter;

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
        double static_friction_increment{1.0}; // Command increment for static friction search (%)
        double static_friction_max{40.0};      // Maximum command to try for static friction
        double min_velocity_threshold{0.01};   // Minimum velocity (rad/s) to consider movement
        std::vector<double> velocity_test_commands{20.0, 40.0, 60.0, 80.0}; // Test speeds (%)
        double velocity_test_duration{2.0};    // Duration per velocity test
        double velocity_settling_time{0.5};    // Time to wait for settling
        int acceleration_test_count{3};        // Number of acceleration tests to average
        double acceleration_test_amplitude{50.0}; // Command for acceleration tests

        // Safety parameters
        double max_test_distance_m{0.6};       // Emergency stop distance
        double max_single_test_duration{15.0}; // Timeout per individual test
        double max_calibration_duration{120.0}; // Total calibration timeout
        int max_retries{3};                    // Retry attempts on failure

        // Validation parameters
        double validation_duration{2.0};       // Duration of validation test
        double validation_max_error{0.2};      // Maximum acceptable tracking error (20%)

        // Value range validation
        struct ValidationRanges {
            double kS_min{5.0};
            double kS_max{30.0};
            double kV_min{0.1};
            double kV_max{2.0};
            double kA_min{0.01};
            double kA_max{0.5};
            double kp_min{0.1};
            double kp_max{10.0};
            double ki_min{0.0};
            double ki_max{5.0};
            double kd_min{0.0};
            double kd_max{2.0};
        } ranges;
    };

    struct CalibrationResult
    {
        foundation::PidGains pid;
        foundation::Feedforward ff;
        bool success{false};
        std::string error_message;
        double duration_seconds{0.0};

        // Detailed metrics for debugging
        struct Metrics {
            double static_friction_forward{0.0};
            double static_friction_backward{0.0};
            double velocity_constant_r_squared{0.0}; // Linear regression fit quality
            int velocity_samples{0};
            double acceleration_mean{0.0};
            double acceleration_std_dev{0.0};

            // PID tuning metrics
            double time_constant_tau{0.0};
            double steady_state_gain{0.0};
            double delay{0.0};

            // Relay feedback metrics (if used)
            double ultimate_gain_ku{0.0};
            double ultimate_period_tu{0.0};
            int oscillation_count{0};

            // Validation metrics
            double validation_mean_error{0.0};
            double validation_max_error{0.0};
            bool validation_passed{false};
        } metrics;
    };

    class MotorCalibrator
    {
    public:
        MotorCalibrator(hal::motor::Motor& motor, const CalibrationConfig& config = {});
        ~MotorCalibrator() = default;

        // Main calibration entry point
        CalibrationResult calibrate();

        // Individual calibration steps (can be called separately)
        foundation::Feedforward calibrateFeedforward();
        foundation::PidGains calibratePID(const foundation::Feedforward& ff);

    private:
        // Feedforward calibration helpers
        double findStaticFriction();
        double findVelocityConstant();
        double findAccelerationConstant(double kS, double kV);

        // PID tuning methods
        foundation::PidGains stepResponseTuning(const foundation::Feedforward& ff);
        foundation::PidGains relayFeedbackTuning(const foundation::Feedforward& ff);

        // Data collection structures
        struct VelocityDataPoint {
            double time;
            double velocity;
            double command;
        };

        struct VelocityProfile {
            std::vector<VelocityDataPoint> data;
            double mean_velocity{0.0};
            double std_dev{0.0};
        };

        // Data recording functions
        VelocityProfile recordVelocityProfile(double command_percent, double duration);
        double measureSteadyStateVelocity(double command_percent, double duration);

        // Analysis functions
        struct LinearRegression {
            double slope;
            double intercept;
            double r_squared;
        };
        LinearRegression linearFit(const std::vector<double>& x, const std::vector<double>& y);

        struct StepResponseParams {
            double tau;        // Time constant
            double K;          // Steady-state gain
            double delay;      // Dead time
            double fit_error;  // RMS fit error
        };
        StepResponseParams fitStepResponse(const VelocityProfile& profile, double initial_velocity);

        struct RelayParams {
            double Ku;         // Ultimate gain
            double Tu;         // Ultimate period
            double amplitude;  // Oscillation amplitude
            int cycles;        // Number of complete cycles
        };
        RelayParams analyzeOscillations(const VelocityProfile& profile);

        // Validation
        bool validateCalibration(const foundation::PidGains& pid,
                                const foundation::Feedforward& ff,
                                CalibrationResult::Metrics& metrics);

        // Safety checks
        bool checkSafetyLimits();
        bool checkTimeout(double start_time);
        bool validateParameterRanges(const foundation::PidGains& pid,
                                     const foundation::Feedforward& ff);

        // Utility functions
        void resetMotor();
        void stopMotor();
        double getCurrentTime() const;
        double getMotorVelocity() const;
        void setMotorCommand(double percent);

        // Member variables
        hal::motor::Motor& motor_;
        CalibrationConfig config_;
        CalibrationResult result_;
        double calibration_start_time_{0.0};
        std::unique_ptr<MotorAdapter> adapter_;

        // Safety state
        double total_distance_moved_{0.0};
        bool emergency_stop_{false};
    };

} // namespace libstp::drive
