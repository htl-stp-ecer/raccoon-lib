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
    /** Per-axis chassis velocity control settings. */
    struct AxisVelocityControlConfig {
        foundation::PidGains pid{0.0, 0.0, 0.0};
        foundation::Feedforward ff{0.0, 1.0, 0.0};  // kV=1.0 = passthrough
    };

    /** Bundle of independent velocity-control settings for `vx`, `vy`, and `wz`. */
    struct ChassisVelocityControlConfig {
        AxisVelocityControlConfig vx{};
        AxisVelocityControlConfig vy{};
        AxisVelocityControlConfig wz{};
    };

    /**
     * Chassis-space drive controller.
     *
     * `Drive` closes the loop on desired body velocity by combining feedback
     * from a kinematics model and an IMU, then forwarding corrected commands to
     * the drivetrain-specific `IKinematics` implementation.
     */
    class Drive final
    {
    public:
        /**
         * @param kinematics Owned drivetrain kinematics implementation.
         * @param vel_config Initial per-axis velocity-controller gains.
         * @param imu IMU used for yaw-rate feedback.
         */
        Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
              const ChassisVelocityControlConfig& vel_config,
              hal::imu::IIMU& imu);

        /** Set the desired body-frame velocity for the next control update. */
        void setVelocity(const foundation::ChassisVelocity& v_body);

        /** Run one chassis velocity control step and send the corrected command to kinematics. */
        kinematics::MotorCommands update(double dt);

        /** Estimate current body-frame velocity using kinematics plus the last IMU yaw rate. */
        [[nodiscard]] foundation::ChassisVelocity estimateState() const;

        /** Forward the modeled wheel count from the underlying kinematics. */
        [[nodiscard]] std::size_t wheelCount() const;

        /** Request a controlled stop by zeroing the desired velocity. */
        void softStop();

        /** Immediately clear controller state and brake the drivetrain. */
        void hardStop();

        /** Replace the per-axis velocity-control settings and rebuild the controllers. */
        void setVelocityControlConfig(const ChassisVelocityControlConfig& config);
        [[nodiscard]] const ChassisVelocityControlConfig& getVelocityControlConfig() const { return vel_ctrl_config_; }

        /** Clear integrator and derivative history in all three velocity controllers. */
        void resetVelocityControllers();

        /** Access the owned kinematics implementation for advanced integration. */
        [[nodiscard]] kinematics::IKinematics& getKinematics() { return *kinematics_; }
        [[nodiscard]] const kinematics::IKinematics& getKinematics() const { return *kinematics_; }

        /** Convenience wrapper around `IKinematics::getWheelRadius()`. */
        [[nodiscard]] double getWheelRadius() const { return kinematics_->getWheelRadius(); }

        /** Convenience wrapper around `IKinematics::getMotors()`. */
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
