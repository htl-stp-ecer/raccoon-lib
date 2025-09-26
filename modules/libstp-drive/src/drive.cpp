//
// Created by tobias on 9/5/25.
//
#include "drive/drive.hpp"
#include <stdexcept>
#include <algorithm>

using namespace libstp::drive;

Drive::Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
             const std::vector<hal::motor::Motor*>& motors)
    : kinematics_(std::move(kinematics))
{
    if (!kinematics_) throw std::invalid_argument("Kinematics pointer cannot be null");
    if (motors.size() != kinematics_->wheelCount())
        throw std::invalid_argument("Number of motors must match number of wheels in kinematics");

    wheels_.reserve(motors.size());
    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        Wheel w{
            .adapter = MotorAdapter{motors[i]},
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
    a.kinematic_sat_mask = 0;
    a.actuator_sat_mask = 0;

    for (std::size_t i = 0; i < wheels_.size(); ++i)
    {
        const double unclamped_target = wheel_targets[i];
        const double target = std::clamp(unclamped_target, -wheel_lim_.max_w, wheel_lim_.max_w);
        if (std::abs(unclamped_target) > wheel_lim_.max_w)
        {
            a.saturated_any = true;
            a.kinematic_sat_mask |= 1u << i;
        }

        double a_ref = 0.0;
        const double limited = wheels_[i].limiter.step(target, wheels_[i].target_w, dt, a_ref);
        wheels_[i].target_w = limited;

        bool motor_sat = false;
        wheels_[i].adapter.setVelocityWithAccel(limited, a_ref, dt, &motor_sat);
        if (motor_sat)
        {
            a.saturated_any = true;
            a.actuator_sat_mask |= 1u << i;
        }
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
