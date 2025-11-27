//
// Created by tobias on 9/5/25.
//
#include "drive/drive.hpp"
#include "foundation/config.hpp"
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
    LIBSTP_LOG_INFO("Drive::setVelocity request vx={}, vy={}, w={}", v_body.vx, v_body.vy, v_body.w);

    const double clamped_vx = std::clamp(v_body.vx, -chassis_lim_.max_v, chassis_lim_.max_v);
    const double clamped_vy = std::clamp(v_body.vy, -chassis_lim_.max_v, chassis_lim_.max_v);
    const double clamped_w = std::clamp(v_body.w, -chassis_lim_.max_omega, chassis_lim_.max_omega);

    const bool limited = (clamped_vx != v_body.vx) || (clamped_vy != v_body.vy) || (clamped_w != v_body.w);

    desired_.vx = clamped_vx;
    desired_.vy = clamped_vy;
    desired_.w = clamped_w;

    LIBSTP_LOG_INFO(
        "Drive::setVelocity stored vx={}, vy={}, w={} (limited={})",
        desired_.vx,
        desired_.vy,
        desired_.w,
        limited);
}

libstp::kinematics::MotorCommands Drive::update(const double dt) const
{
    const foundation::ChassisCmd cmd{desired_.vx, desired_.vy, desired_.w};
    LIBSTP_LOG_INFO("Drive::update dt={} -> applying cmd vx={}, vy={}, wz={}", dt, cmd.vx, cmd.vy, cmd.wz);
    LIBSTP_LOG_TRACE("Drive::update target wheel count={} (if available)", kinematics_->wheelCount());

    return kinematics_->applyCommand(cmd, dt);
}

libstp::foundation::ChassisState Drive::estimateState() const
{
    const auto state = kinematics_->estimateState();
    LIBSTP_LOG_TRACE("Drive::estimateState -> vx={}, vy={}, w={}", state.vx, state.vy, state.wz);
    return state;
}

std::size_t Drive::wheelCount() const
{
    return kinematics_->wheelCount();
}

void Drive::softStop()
{
    LIBSTP_LOG_INFO("Drive::softStop invoked; zeroing desired velocity");
    setVelocity(foundation::ChassisVel(0, 0, 0));
}


void Drive::hardStop()
{
    LIBSTP_LOG_INFO("Drive::hardStop invoked; zeroing desired velocity");
    desired_ = foundation::ChassisVel{0, 0, 0};
    kinematics_->hardStop();
}
