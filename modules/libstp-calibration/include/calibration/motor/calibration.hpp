#pragma once

#include "foundation/motor.hpp"
#include "hal/IMotor.hpp"
#include <memory>

#include "calibration_config.hpp"
#include "calibration_result.hpp"

// Forward declarations for new component structure
namespace libstp::calibration
{
    namespace motor { class MotorControlInterface; }
    namespace data { class VelocityProfileRecorder; }
    namespace feedforward {
        class StaticFrictionCalibrator;
        class VelocityConstantCalibrator;
        class AccelerationConstantCalibrator;
    }
    namespace pid {
        class StepResponseTuner;
        class RelayFeedbackTuner;
    }
    namespace validation {
        class CalibrationValidator;
        class SafetyMonitor;
    }
}

namespace libstp::calibration
{
    /**
     * @brief Motor calibration orchestrator
     *
     * Coordinates feedforward and PID calibration using specialized components.
     * This class is now a thin orchestration layer over focused components.
     */
    class MotorCalibrator
    {
    public:
        explicit MotorCalibrator(hal::motor::IMotor& motor, CalibrationConfig config = {});
        ~MotorCalibrator();

        /** Run the full feedforward + PID calibration pipeline. */
        CalibrationResult calibrate();

        /** Measure static friction, velocity, and acceleration feedforward terms. */
        foundation::Feedforward calibrateFeedforward();
        /** Tune PID gains using the configured strategy and a known feedforward model. */
        foundation::PidGains calibratePID(const foundation::Feedforward& ff);

    private:
        // Configuration and results
        CalibrationConfig config_;
        CalibrationResult result_;
        double calibration_start_time_{0.0};
        bool emergency_stop_{false};
        double total_distance_moved_{0.0};

        // Component instances (using forward declarations)
        std::unique_ptr<motor::MotorControlInterface> motor_control_;
        std::unique_ptr<data::VelocityProfileRecorder> recorder_;
        std::unique_ptr<feedforward::StaticFrictionCalibrator> static_friction_cal_;
        std::unique_ptr<feedforward::VelocityConstantCalibrator> velocity_cal_;
        std::unique_ptr<feedforward::AccelerationConstantCalibrator> acceleration_cal_;
        std::unique_ptr<pid::StepResponseTuner> step_tuner_;
        std::unique_ptr<pid::RelayFeedbackTuner> relay_tuner_;
        std::unique_ptr<validation::CalibrationValidator> validator_;
        std::unique_ptr<validation::SafetyMonitor> safety_monitor_;
    };

} // namespace libstp::calibration
