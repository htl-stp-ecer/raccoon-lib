//
// Created by tobias on 9/5/25.
//

#pragma once
#include <memory>
#include <vector>

#include "kinematics/kinematics.hpp"
#include "foundation/types.hpp"
#include "drive/velocity_controller.hpp"
#include "foundation/config.hpp"
#include "hal/IMotor.hpp"

namespace libstp::hal::imu { struct IIMU; }

namespace libstp::drive
{
    struct AxisVelocityControlConfig {
        foundation::PidGains pid{0.0, 0.0, 0.0};
        foundation::Feedforward ff{0.0, 1.0, 0.0};  // kV=1.0 = passthrough
    };

    struct ChassisVelocityControlConfig {
        AxisVelocityControlConfig vx{};
        AxisVelocityControlConfig vy{};
        AxisVelocityControlConfig wz{};
    };

    class Drive final
    {
    public:
        Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
              const ChassisVelocityControlConfig& vel_config,
              hal::imu::IIMU& imu);

        void setVelocity(const foundation::ChassisVelocity& v_body);
        kinematics::MotorCommands update(double dt);

        [[nodiscard]] foundation::ChassisVelocity estimateState() const;
        [[nodiscard]] std::size_t wheelCount() const;

        void softStop();
        void hardStop();

        void setVelocityControlConfig(const ChassisVelocityControlConfig& config);
        [[nodiscard]] const ChassisVelocityControlConfig& getVelocityControlConfig() const { return vel_ctrl_config_; }
        void resetVelocityControllers();

        [[nodiscard]] kinematics::IKinematics& getKinematics() { return *kinematics_; }
        [[nodiscard]] const kinematics::IKinematics& getKinematics() const { return *kinematics_; }

        [[nodiscard]] double getWheelRadius() const { return kinematics_->getWheelRadius(); }

        [[nodiscard]] std::vector<hal::motor::IMotor*> getMotors() const { return kinematics_->getMotors(); }

    private:
        void initControllers();

        std::unique_ptr<kinematics::IKinematics> kinematics_;

        foundation::ChassisVelocity desired_{};

        // Velocity control
        ChassisVelocityControlConfig vel_ctrl_config_{};
        hal::imu::IIMU& imu_;

        VelocityController ctrl_vx_;
        VelocityController ctrl_vy_;
        VelocityController ctrl_wz_;

        // Last gyro wz reading for estimateState()
        float last_gyro_wz_{0.0f};
    };
}
