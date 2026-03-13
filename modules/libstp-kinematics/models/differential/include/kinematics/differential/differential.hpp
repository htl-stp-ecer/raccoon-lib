//
// Created by tobias on 9/8/25.
//

#pragma once
#include "kinematics/kinematics.hpp"
#include "drive/motor_adapter.hpp"
#include "hal/IMotor.hpp"
#include <vector>

namespace libstp::calibration {
    struct CalibrationConfig;
    struct CalibrationResult;
}

namespace libstp::kinematics::differential
{
    /**
     * Differential-drive implementation of `IKinematics`.
     *
     * The model assumes two independently driven wheels separated by
     * `m_wheelbase`, with no direct lateral degree of freedom.
     */
    class DifferentialKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_wheelRadius;
        double m_maxWheelSpeed{0.0}; // 0 = no limiting

        drive::MotorAdapter left_motor_;
        drive::MotorAdapter right_motor_;

    public:
        /**
         * @param left_motor Non-owning pointer to the left drive motor.
         * @param right_motor Non-owning pointer to the right drive motor.
         * @param wheelbase Distance between left and right wheel contact lines.
         * @param wheelRadius Radius of each drive wheel in meters.
         */
        DifferentialKinematics(hal::motor::IMotor* left_motor,
                               hal::motor::IMotor* right_motor,
                               double wheelbase,
                               double wheelRadius);
        ~DifferentialKinematics() override = default;

        /** Always returns `2`. */
        [[nodiscard]] std::size_t wheelCount() const override;

        /** Convert `(vx, wz)` into left and right wheel angular velocities. */
        MotorCommands applyCommand(const foundation::ChassisVelocity& cmd, double dt) override;

        /** Recover `(vx, wz)` from the measured left and right wheel speeds. */
        [[nodiscard]] foundation::ChassisVelocity estimateState() const override;

        /** Brake both drive motors immediately. */
        void hardStop() override;

        /** Differential drivetrains cannot command lateral motion directly. */
        [[nodiscard]] bool supportsLateralMotion() const override;

        /** Clear encoder history in both `MotorAdapter` instances. */
        void resetEncoders() override;

        /** Forward the default calibration flow inherited from `IKinematics`. */
        using kinematics::IKinematics::calibrateMotors;

        /** Calibrate left then right motor with a short dwell in between. */
        std::vector<calibration::CalibrationResult> calibrateMotors(const calibration::CalibrationConfig& config) override;

        [[nodiscard]] double getWheelRadius() const override { return m_wheelRadius; }

        /** Return the underlying motors in left, right order. */
        [[nodiscard]] std::vector<hal::motor::IMotor*> getMotors() const override;

        /** Command motors at raw open-loop power using differential inverse kinematics for direction. */
        void applyPowerCommand(const foundation::ChassisVelocity& direction,
                               int power_percent) override;

        void setMaxWheelSpeed(double max_wheel_speed) { m_maxWheelSpeed = max_wheel_speed; }
        [[nodiscard]] double getMaxWheelSpeed() const { return m_maxWheelSpeed; }
    };
}
