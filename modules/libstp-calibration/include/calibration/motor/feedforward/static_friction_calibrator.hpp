#pragma once

#include "calibration/motor/calibration_config.hpp"

namespace libstp::calibration::motor
{
    class MotorControlInterface;
}

namespace libstp::calibration::feedforward
{
    class StaticFrictionCalibrator
    {
    public:
        StaticFrictionCalibrator(
            motor::MotorControlInterface& motor,
            const CalibrationConfig& config
        );

        double findStaticFriction(
            double& forward_threshold,
            double& backward_threshold,
            double start_time
        );

    private:
        motor::MotorControlInterface& motor_;
        const CalibrationConfig& config_;
    };
}
