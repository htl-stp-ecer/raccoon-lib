#pragma once

#include "velocity_data.hpp"

namespace libstp::calibration::motor
{
    class MotorControlInterface;
}

namespace libstp::calibration::data
{
    class VelocityProfileRecorder
    {
    public:
        explicit VelocityProfileRecorder(motor::MotorControlInterface& motor);

        VelocityProfile recordProfile(
            double command_percent,
            double duration,
            bool& emergency_stop,
            double start_time
        );

        double measureSteadyStateVelocity(
            double command_percent,
            double duration,
            bool& emergency_stop,
            double start_time
        );

    private:
        motor::MotorControlInterface& motor_;
    };
}
