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
    class VelocityConstantCalibrator
    {
    public:
        VelocityConstantCalibrator(
            motor::MotorControlInterface& motor,
            data::VelocityProfileRecorder& recorder,
            const CalibrationConfig& config
        );

        double findVelocityConstant(
            double& kS_from_regression,
            double& r_squared,
            int& sample_count,
            double start_time,
            bool& emergency_stop
        );

    private:
        motor::MotorControlInterface& motor_;
        data::VelocityProfileRecorder& recorder_;
        const CalibrationConfig& config_;
    };
}
