//
// Created by tobias on 9/5/25.
//
#include "drive/drive.hpp"
#include "hal/IIMU.hpp"
#include "foundation/config.hpp"
#include <stdexcept>
#include <limits>

using namespace libstp::drive;

Drive::Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
             const ChassisVelocityControlConfig& vel_config,
             hal::imu::IIMU& imu)
    : kinematics_(std::move(kinematics)),
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
    desired_ = v_body;

    LIBSTP_LOG_TRACE("Drive::setVelocity vx={}, vy={}, wz={}",
        desired_.vx, desired_.vy, desired_.wz);
}

libstp::kinematics::MotorCommands Drive::update(const double dt)
{
    const auto measured_enc = kinematics_->estimateState();

    last_gyro_wz_ = imu_.getYawRate();

    constexpr double kNoLimit = std::numeric_limits<double>::infinity();
    const double corrected_vx = ctrl_vx_.compute(desired_.vx, 0.0, measured_enc.vx, dt, kNoLimit);
    const double corrected_vy = ctrl_vy_.compute(desired_.vy, 0.0, measured_enc.vy, dt, kNoLimit);
    const double corrected_wz = ctrl_wz_.compute(desired_.wz, 0.0, static_cast<double>(last_gyro_wz_), dt, kNoLimit);

    const foundation::ChassisVelocity corrected{corrected_vx, corrected_vy, corrected_wz};

    LIBSTP_LOG_DEBUG(
        "DRIVE dt={:.4f} des_wz={:.3f} gyro_wz={:.3f} enc_wz={:.3f} corr_wz={:.3f} | "
        "des_vx={:.3f} enc_vx={:.3f} corr_vx={:.3f}",
        dt,
        desired_.wz, last_gyro_wz_, measured_enc.wz, corrected.wz,
        desired_.vx, measured_enc.vx, corrected.vx);

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
