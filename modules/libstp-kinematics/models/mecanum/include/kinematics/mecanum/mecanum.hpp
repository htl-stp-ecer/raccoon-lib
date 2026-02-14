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
    class MecanumKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_trackWidth;
        double m_wheelRadius;

        drive::MotorAdapter front_left_motor_;
        drive::MotorAdapter front_right_motor_;
        drive::MotorAdapter back_left_motor_;
        drive::MotorAdapter back_right_motor_;

    public:
        MecanumKinematics(hal::motor::IMotor* front_left_motor,
                         hal::motor::IMotor* front_right_motor,
                         hal::motor::IMotor* back_left_motor,
                         hal::motor::IMotor* back_right_motor,
                         double wheelbase,
                         double trackWidth,
                         double wheelRadius);
        ~MecanumKinematics() override = default;

        [[nodiscard]] std::size_t wheelCount() const override;
        MotorCommands applyCommand(const foundation::ChassisVelocity& cmd, double dt) override;
        [[nodiscard]] foundation::ChassisVelocity estimateState() const override;
        void hardStop() override;
        [[nodiscard]] bool supportsLateralMotion() const override;
        void resetEncoders() override;

        // Calibration methods
        using kinematics::IKinematics::calibrateMotors;
        std::vector<calibration::CalibrationResult> calibrateMotors(const calibration::CalibrationConfig& config) override;

        [[nodiscard]] double getWheelRadius() const override { return m_wheelRadius; }

        [[nodiscard]] std::vector<hal::motor::IMotor*> getMotors() const override;
    };
}
