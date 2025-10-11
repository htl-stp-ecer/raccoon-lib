//
// Created by tobias on 9/5/25.
//
#include "drive/drive.hpp"
#include <stdexcept>
#include <algorithm>

using namespace libstp::drive;

Drive::Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
             const MotionLimits& chassis_lim)
    : kinematics_(std::move(kinematics))
{
    if (!kinematics_) throw std::invalid_argument("Kinematics pointer cannot be null");
    chassis_lim_ = chassis_lim;
}


void Drive::setVelocity(const foundation::ChassisVel& v_body)
{
    desired_.vx = std::clamp(v_body.vx, -chassis_lim_.max_v, chassis_lim_.max_v);
    desired_.vy = std::clamp(v_body.vy, -chassis_lim_.max_v, chassis_lim_.max_v);
    desired_.w = std::clamp(v_body.w, -chassis_lim_.max_omega, chassis_lim_.max_omega);
}

void Drive::update(const double dt) const
{
    // Apply the chassis velocity command through the kinematics
    kinematics_->applyCommand(foundation::ChassisCmd{desired_.vx, desired_.vy, desired_.w}, dt);
}

libstp::foundation::ChassisState Drive::estimateState() const
{
    return kinematics_->estimateState();
}

std::size_t Drive::wheelCount() const
{
    return kinematics_->wheelCount();
}

void Drive::hardStop()
{
    desired_ = foundation::ChassisVel{0, 0, 0};
    kinematics_->hardStop();
}
