//
// Created by tobias on 9/5/25.
//
#include "drive/drive.hpp"
#include "hal/IIMU.hpp"
#include "foundation/config.hpp"
#include <stdexcept>
#include <algorithm>

using namespace libstp::drive;

Drive::Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
             const MotionLimits& chassis_lim,
             const ChassisVelocityControlConfig& vel_config,
             hal::imu::IIMU& imu)
    : kinematics_(std::move(kinematics)),
      chassis_lim_(chassis_lim),
      vel_ctrl_config_(vel_config),
      imu_(imu),
      ctrl_vx_(vel_config.vx.pid, vel_config.vx.ff),
      ctrl_vy_(vel_config.vy.pid, vel_config.vy.ff),
      ctrl_wz_(vel_config.wz.pid, vel_config.wz.ff)
{
    if (!kinematics_) throw std::invalid_argument("Kinematics pointer cannot be null");
}

void Drive::initControllers()
{
    ctrl_vx_ = VelocityController(vel_ctrl_config_.vx.pid, vel_ctrl_config_.vx.ff);
    ctrl_vy_ = VelocityController(vel_ctrl_config_.vy.pid, vel_ctrl_config_.vy.ff);
    ctrl_wz_ = VelocityController(vel_ctrl_config_.wz.pid, vel_ctrl_config_.wz.ff);
    LIBSTP_LOG_DEBUG("Drive: velocity controllers initialized");
}

void Drive::setVelocity(const foundation::ChassisVelocity& v_body)
{
    LIBSTP_LOG_TRACE("Drive::setVelocity request vx={}, vy={}, wz={}", v_body.vx, v_body.vy, v_body.wz);

    const double clamped_vx = std::clamp(v_body.vx, -chassis_lim_.max_v, chassis_lim_.max_v);
    const double clamped_vy = std::clamp(v_body.vy, -chassis_lim_.max_v, chassis_lim_.max_v);
    const double clamped_wz = std::clamp(v_body.wz, -chassis_lim_.max_omega, chassis_lim_.max_omega);

    const bool limited = (clamped_vx != v_body.vx) || (clamped_vy != v_body.vy) || (clamped_wz != v_body.wz);

    desired_.vx = clamped_vx;
    desired_.vy = clamped_vy;
    desired_.wz = clamped_wz;

    LIBSTP_LOG_TRACE(
        "Drive::setVelocity stored vx={}, vy={}, wz={} (limited={})",
        desired_.vx,
        desired_.vy,
        desired_.wz,
        limited);
}

libstp::kinematics::MotorCommands Drive::update(const double dt)
{
    const auto measured_enc = kinematics_->estimateState();

    last_gyro_wz_ = imu_.getYawRate();

    const double corrected_vx = ctrl_vx_.compute(desired_.vx, 0.0, measured_enc.vx, dt, chassis_lim_.max_v);
    const double corrected_vy = ctrl_vy_.compute(desired_.vy, 0.0, measured_enc.vy, dt, chassis_lim_.max_v);
    const double corrected_wz = ctrl_wz_.compute(desired_.wz, 0.0, static_cast<double>(last_gyro_wz_), dt, chassis_lim_.max_omega);

    const foundation::ChassisVelocity corrected{corrected_vx, corrected_vy, corrected_wz};

    LIBSTP_LOG_TRACE(
        "Drive::update dt={} desired=({},{},{}) measured_enc=({},{},{}) gyro_wz={} corrected=({},{},{})",
        dt,
        desired_.vx, desired_.vy, desired_.wz,
        measured_enc.vx, measured_enc.vy, measured_enc.wz,
        last_gyro_wz_,
        corrected.vx, corrected.vy, corrected.wz);

    return kinematics_->applyCommand(corrected, dt);
}

libstp::foundation::ChassisVelocity Drive::estimateState() const
{
    auto state = kinematics_->estimateState();
    state.wz = static_cast<double>(last_gyro_wz_);

    LIBSTP_LOG_TRACE("Drive::estimateState -> vx={}, vy={}, wz={}", state.vx, state.vy, state.wz);
    return state;
}

std::size_t Drive::wheelCount() const
{
    return kinematics_->wheelCount();
}

void Drive::softStop()
{
    LIBSTP_LOG_TRACE("Drive::softStop invoked; zeroing desired velocity");
    setVelocity(foundation::ChassisVelocity(0, 0, 0));
}

void Drive::hardStop()
{
    LIBSTP_LOG_DEBUG("Drive::hardStop invoked; zeroing desired velocity");
    desired_ = foundation::ChassisVelocity{0, 0, 0};
    resetVelocityControllers();
    kinematics_->hardStop();
}

void Drive::setVelocityControlConfig(const ChassisVelocityControlConfig& config)
{
    vel_ctrl_config_ = config;
    initControllers();
    LIBSTP_LOG_DEBUG("Drive: velocity control config updated");
}

void Drive::resetVelocityControllers()
{
    ctrl_vx_.reset();
    ctrl_vy_.reset();
    ctrl_wz_.reset();
    LIBSTP_LOG_TRACE("Drive: velocity controllers reset");
}
