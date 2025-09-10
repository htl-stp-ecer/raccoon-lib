//
// Created by tobias on 9/5/25.
//
#include "drive/drive.hpp"
#include <stdexcept>
#include <algorithm>

using namespace libstp::drive;

Drive::Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
             const std::vector<hal::motor::Motor>& motors,
             const std::vector<MotorCalibration>& calibrations)
    : kinematics_(std::move(kinematics))
{
    if (!kinematics_) throw std::invalid_argument("Kinematics pointer cannot be null");
    if (motors.size() != kinematics_->wheelCount())
        throw std::invalid_argument("Number of motors must match number of wheels in kinematics");
    if (!calibrations.empty() && calibrations.size() != motors.size())
        throw std::invalid_argument("Calibrations vector must be empty or match number of motors");

    wheels_.reserve(motors.size());
    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        const auto& calibration = calibrations.empty() ? MotorCalibration{} : calibrations[i];
        Wheel w{
            .adapter = MotorAdapter{motors[i], calibration},
        };
        wheels_.push_back(std::move(w));
    }
}

void Drive::setChassisLimits(const MotionLimits& lim) { chassis_lim_ = lim; }

void Drive::setWheelLimits(const WheelLimits& lim)
{
    wheel_lim_ = lim;
    for (auto& w : wheels_)
    {
        w.limiter.setMaxRate(std::max(0.0, wheel_lim_.max_w_dot));
    }
}

void Drive::setWheelControllerGains(const VelocityController::Gains& g)
{
    for (auto& w : wheels_)
    {
        auto calibration = w.adapter.getCalibration();
        calibration.pid_gains = g;
        w.adapter.setCalibration(calibration);
    }
}

void Drive::setWheelCalibration(const std::size_t wheel_index, const MotorCalibration& calibration)
{
    if (wheel_index >= wheels_.size())
        throw std::out_of_range("Wheel index out of range");
    wheels_[wheel_index].adapter.setCalibration(calibration);
}

void Drive::setAllWheelCalibrations(const std::vector<MotorCalibration>& calibrations)
{
    if (calibrations.size() != wheels_.size())
        throw std::invalid_argument("Calibrations vector size must match number of wheels");
    for (std::size_t i = 0; i < wheels_.size(); ++i)
    {
        wheels_[i].adapter.setCalibration(calibrations[i]);
    }
}

void Drive::setVelocity(const foundation::ChassisVel& v_body)
{
    desired_.vx = std::clamp(v_body.vx, -chassis_lim_.max_v, chassis_lim_.max_v);
    desired_.vy = std::clamp(v_body.vy, -chassis_lim_.max_v, chassis_lim_.max_v);
    desired_.w = std::clamp(v_body.w, -chassis_lim_.max_omega, chassis_lim_.max_omega);
}

Achieved Drive::update(const double dt)
{
    const auto wheel_targets = kinematics_->inverse(foundation::ChassisCmd{desired_.vx, desired_.vy, desired_.w});

    Achieved a{};
    a.saturated_any = false;
    a.saturation_mask = 0;

    for (std::size_t i = 0; i < wheels_.size(); ++i)
    {
        const double target = std::clamp(wheel_targets[i], -wheel_lim_.max_w, wheel_lim_.max_w);
        if (std::abs(wheel_targets[i]) > wheel_lim_.max_w)
        {
            a.saturated_any = true;
            a.saturation_mask |= 1u << i;
        }

        const double limited = wheels_[i].limiter.step(target, wheels_[i].target_w, dt);
        wheels_[i].target_w = limited;
        wheels_[i].adapter.setVelocity(limited, dt);
    }

    std::vector meas_w(wheels_.size(), 0.0);
    for (std::size_t i = 0; i < wheels_.size(); ++i)
    {
        meas_w[i] = wheels_[i].adapter.getVelocity();
    }
    const auto [vx, vy, wz] = kinematics_->forward(meas_w);
    a.body = foundation::ChassisVel{vx, vy, wz};

    return a;
}

libstp::foundation::ChassisState Drive::estimateState() const
{
    std::vector meas_w(wheels_.size(), 0.0);
    for (std::size_t i = 0; i < wheels_.size(); ++i)
        meas_w[i] = wheels_[i].adapter.getVelocity();
    return kinematics_->forward(meas_w);
}

std::size_t Drive::wheelCount() const { return kinematics_->wheelCount(); }

void Drive::stop(const bool hard)
{
    desired_ = foundation::ChassisVel{0, 0, 0};
    if (hard)
    {
        for (auto& w : wheels_)
        {
            w.target_w = 0.0;
            w.adapter.resetController();
            w.adapter.brake();
        }
    }
}
