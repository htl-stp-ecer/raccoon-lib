#pragma once

#include "foundation/motor.hpp"
#include "calibration/motor/calibration_config.hpp"

namespace libstp::calibration::motor
{
    class MotorControlInterface;
}

namespace libstp::calibration::data
{
    class VelocityProfileRecorder;
}

namespace libstp::calibration::pid
{
    class StepResponseTuner
    {
    public:
        StepResponseTuner(
            motor::MotorControlInterface& motor,
            data::VelocityProfileRecorder& recorder,
            const CalibrationConfig& config
        );

        foundation::PidGains tune(
            const foundation::Feedforward& ff,
            double& tau,
            double& steady_state_gain,
            double& delay,
            double start_time,
            bool& emergency_stop
        );

    private:
        motor::MotorControlInterface& motor_;
        data::VelocityProfileRecorder& recorder_;
        const CalibrationConfig& config_;
    };
}
