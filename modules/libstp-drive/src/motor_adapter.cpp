//
// Created by tobias on 9/8/25.
//
#include "drive/motor_adapter.hpp"
#include <algorithm>
#include <cmath>

using namespace libstp::drive;

MotorAdapter::MotorAdapter(const hal::motor::Motor& motor, const MotorCalibration& calibration)
    : motor_(motor), calibration_(calibration), controller_(calibration.pid_gains)
{
}

void MotorAdapter::setVelocity(const double target_rad_per_s, const double dt)
{
    last_target_velocity_ = target_rad_per_s;
    velocity_control_active_ = true;

    const double current_velocity = getVelocity();

    const double pid_output = controller_.compute(target_rad_per_s, current_velocity, dt);

    double percent_output = pid_output * calibration_.velocity_to_percent_scale;
    percent_output = std::clamp(percent_output, -calibration_.max_percent_output, calibration_.max_percent_output);

    motor_.setSpeed(static_cast<int>(std::round(percent_output)));
}

double MotorAdapter::getVelocity() const
{
    const int raw_percent = motor_.getSpeed();
    const double adjusted_percent = static_cast<double>(raw_percent) - calibration_.bemf_offset;
    return adjusted_percent * calibration_.percent_to_velocity_scale;
}

void MotorAdapter::setPercent(const double percent)
{
    velocity_control_active_ = false;
    const double clamped_percent = std::clamp(percent, -calibration_.max_percent_output,
                                              calibration_.max_percent_output);
    motor_.setSpeed(static_cast<int>(std::round(clamped_percent)));
}

int MotorAdapter::getRawPercent() const
{
    return motor_.getSpeed();
}

void MotorAdapter::setCalibration(const MotorCalibration& calibration)
{
    calibration_ = calibration;
    controller_.setGains(calibration.pid_gains);
}

const MotorCalibration& MotorAdapter::getCalibration() const
{
    return calibration_;
}

void MotorAdapter::resetController()
{
    controller_.reset();
    velocity_control_active_ = false;
}

void MotorAdapter::brake()
{
    motor_.brake();
    resetController();
}

libstp::hal::motor::Motor& MotorAdapter::getMotor()
{
    return motor_;
}

const libstp::hal::motor::Motor& MotorAdapter::getMotor() const
{
    return motor_;
}
