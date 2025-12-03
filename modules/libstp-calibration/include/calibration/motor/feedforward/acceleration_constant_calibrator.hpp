#pragma once

#include "calibration/motor/calibration_config.hpp"

namespace libstp::calibration::motor
{
    class MotorControlInterface;
}

namespace libstp::calibration::data
{
    class VelocityProfileRecorder;
}

namespace libstp::calibration::feedforward
{
    class AccelerationConstantCalibrator
    {
    public:
        AccelerationConstantCalibrator(
            motor::MotorControlInterface& motor,
            data::VelocityProfileRecorder& recorder,
            const CalibrationConfig& config
        );

        double findAccelerationConstant(
            double kS,
            double kV,
            double& mean_value,
            double& std_dev,
            double start_time,
            bool& emergency_stop
        );

    private:
        motor::MotorControlInterface& motor_;
        data::VelocityProfileRecorder& recorder_;
        const CalibrationConfig& config_;
    };
}
