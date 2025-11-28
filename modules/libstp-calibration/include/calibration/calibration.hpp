//
// Created by tobias on 27/11/25.
//


#pragma once

#include "foundation/motor.hpp"
#include "hal/Motor.hpp"
#include <vector>
#include <string>
#include <memory>

#include "calibration_config.hpp"
#include "calibration_result.hpp"

namespace libstp::drive
{
    class MotorAdapter;
}

namespace libstp::calibration
{
    class MotorCalibrator
    {
    public:
        explicit MotorCalibrator(hal::motor::Motor& motor, CalibrationConfig  config = {});
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
        static LinearRegression linearFit(const std::vector<double>& x, const std::vector<double>& y);

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
        std::unique_ptr<drive::MotorAdapter> adapter_;

        // Safety state
        double total_distance_moved_{0.0};
        bool emergency_stop_{false};
    };

} // namespace libstp::drive
