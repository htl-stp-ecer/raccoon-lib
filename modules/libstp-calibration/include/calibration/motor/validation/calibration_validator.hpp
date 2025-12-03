#pragma once

#include "foundation/motor.hpp"
#include "calibration/motor/calibration_config.hpp"
#include "calibration/motor/calibration_result.hpp"

namespace libstp::calibration::motor
{
    class MotorControlInterface;
}

namespace libstp::calibration::data
{
    class VelocityProfileRecorder;
}

namespace libstp::calibration::validation
{
    class CalibrationValidator
    {
    public:
        CalibrationValidator(
            motor::MotorControlInterface& motor,
            data::VelocityProfileRecorder& recorder,
            const CalibrationConfig& config
        );

        bool validateCalibration(
            const foundation::PidGains& pid,
            const foundation::Feedforward& ff,
            CalibrationResult::Metrics& metrics,
            double start_time,
            bool& emergency_stop
        );

        bool validateParameterRanges(
            const foundation::PidGains& pid,
            const foundation::Feedforward& ff
        );

    private:
        motor::MotorControlInterface& motor_;
        data::VelocityProfileRecorder& recorder_;
        const CalibrationConfig& config_;
    };
}
