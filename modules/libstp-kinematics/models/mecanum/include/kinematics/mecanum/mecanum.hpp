//
// Created by tobias on 9/8/25.
//

#pragma once
#include "kinematics/kinematics.hpp"
#include "drive/motor_adapter.hpp"
#include "foundation/config.hpp"
#include "hal/IMotor.hpp"
#include <vector>

namespace libstp::calibration {
    struct CalibrationConfig;
    struct CalibrationResult;
}

namespace libstp::kinematics::mecanum
{
    /**
     * Four-wheel mecanum implementation of `IKinematics`.
     *
     * The model uses the shared LibSTP body-frame convention: +x forward,
     * +y right, +wz counter-clockwise.
     */
    class MecanumKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_trackWidth;
        double m_wheelRadius;
        double m_maxWheelSpeed{0.0}; // 0 = no limiting

        drive::MotorAdapter front_left_motor_;
        drive::MotorAdapter front_right_motor_;
        drive::MotorAdapter back_left_motor_;
        drive::MotorAdapter back_right_motor_;

    public:
        /**
         * @param front_left_motor Non-owning pointer to the front-left motor.
         * @param front_right_motor Non-owning pointer to the front-right motor.
         * @param back_left_motor Non-owning pointer to the back-left motor.
         * @param back_right_motor Non-owning pointer to the back-right motor.
         * @param wheelbase Front-to-back axle spacing in meters.
         * @param trackWidth Left-to-right wheel spacing in meters.
         * @param wheelRadius Radius of each drive wheel in meters.
         */
        MecanumKinematics(hal::motor::IMotor* front_left_motor,
                         hal::motor::IMotor* front_right_motor,
                         hal::motor::IMotor* back_left_motor,
                         hal::motor::IMotor* back_right_motor,
                         double wheelbase,
                         double trackWidth,
                         double wheelRadius);
        ~MecanumKinematics() override = default;

        /** Always returns `4`. */
        [[nodiscard]] std::size_t wheelCount() const override;

        /** Convert chassis velocity into front-left, front-right, back-left, back-right wheel speeds. */
        MotorCommands applyCommand(const foundation::ChassisVelocity& cmd, double dt) override;

        /** Recover chassis velocity from the four measured wheel speeds. */
        [[nodiscard]] foundation::ChassisVelocity estimateState() const override;

        /** Brake all four drive motors immediately. */
        void hardStop() override;

        /** Mecanum drivetrains can command lateral motion directly. */
        [[nodiscard]] bool supportsLateralMotion() const override;

        /** Clear encoder history in all four `MotorAdapter` instances. */
        void resetEncoders() override;

        /** Forward the default calibration flow inherited from `IKinematics`. */
        using kinematics::IKinematics::calibrateMotors;

        /** Calibrate all four motors in wheel order with short pauses between motors. */
        std::vector<calibration::CalibrationResult> calibrateMotors(const calibration::CalibrationConfig& config) override;

        [[nodiscard]] double getWheelRadius() const override { return m_wheelRadius; }

        /** Set max wheel speed (rad/s) for desaturation. 0 = disabled. */
        void setMaxWheelSpeed(double max_wheel_speed) { m_maxWheelSpeed = max_wheel_speed; }
        [[nodiscard]] double getMaxWheelSpeed() const { return m_maxWheelSpeed; }

        /** Return the underlying motors in front-left, front-right, back-left, back-right order. */
        [[nodiscard]] std::vector<hal::motor::IMotor*> getMotors() const override;
    };
}
