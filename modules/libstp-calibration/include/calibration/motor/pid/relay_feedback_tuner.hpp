#pragma once

#include "foundation/motor.hpp"
#include "calibration/motor/calibration_config.hpp"

namespace libstp::calibration::motor
{
    class MotorControlInterface;
}

namespace libstp::calibration::pid
{
    class StepResponseTuner;

    class RelayFeedbackTuner
    {
    public:
        RelayFeedbackTuner(
            motor::MotorControlInterface& motor,
            StepResponseTuner& step_tuner,
            const CalibrationConfig& config
        );

        foundation::PidGains tune(
            const foundation::Feedforward& ff,
            double& ultimate_gain,
            double& ultimate_period,
            int& oscillation_count,
            double start_time,
            bool& emergency_stop
        );

    private:
        motor::MotorControlInterface& motor_;
        StepResponseTuner& step_tuner_;
        const CalibrationConfig& config_;
    };
}
