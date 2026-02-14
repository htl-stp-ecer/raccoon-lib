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
    class DifferentialKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_wheelRadius;

        drive::MotorAdapter left_motor_;
        drive::MotorAdapter right_motor_;

    public:
        DifferentialKinematics(hal::motor::IMotor* left_motor,
                               hal::motor::IMotor* right_motor,
                               double wheelbase,
                               double wheelRadius);
        ~DifferentialKinematics() override = default;

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
