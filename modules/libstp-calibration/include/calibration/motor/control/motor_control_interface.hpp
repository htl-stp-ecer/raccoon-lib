#pragma once

#include "hal/IMotor.hpp"
#include <memory>

namespace libstp::drive
{
    class MotorAdapter;
}

namespace libstp::calibration::motor
{
    class MotorControlInterface
    {
    public:
        explicit MotorControlInterface(hal::motor::IMotor& motor);
        ~MotorControlInterface();

        void setCommand(double percent);
        void stop();
        void reset();

        double getVelocity() const;
        void updateEncoderVelocity(double dt);

        drive::MotorAdapter& getAdapter() { return *adapter_; }
        const drive::MotorAdapter& getAdapter() const { return *adapter_; }

    private:
        hal::motor::IMotor& motor_;
        std::unique_ptr<drive::MotorAdapter> adapter_;
    };
}
