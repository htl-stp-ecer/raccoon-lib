#include "calibration/motor/control/motor_control_interface.hpp"
#include "drive/motor_adapter.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include <chrono>
#include <thread>
#include <cmath>

namespace libstp::calibration::motor
{
    MotorControlInterface::MotorControlInterface(hal::motor::IMotor& motor)
        : motor_(motor)
    {
        adapter_ = std::make_unique<drive::MotorAdapter>(&motor_);
    }

    MotorControlInterface::~MotorControlInterface() = default;

    void MotorControlInterface::setCommand(double percent)
    {
        int command = static_cast<int>(std::round(utils::clamp(percent, -100.0, 100.0)));
        motor_.setSpeed(command);
    }

    void MotorControlInterface::stop()
    {
        motor_.setSpeed(0);
    }

    void MotorControlInterface::reset()
    {
        stop();

        // Wait for motor to fully stop - check velocity repeatedly
        for (int i = 0; i < 10; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            adapter_->updateEncoderVelocity(0.05);

            double velocity = std::abs(getVelocity());
            if (velocity < 0.1) {
                break;  // Motor has stopped
            }
        }

        // Additional settling time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    double MotorControlInterface::getVelocity() const
    {
        return adapter_->getVelocity();
    }

    void MotorControlInterface::updateEncoderVelocity(double dt)
    {
        adapter_->updateEncoderVelocity(dt);
    }
}
