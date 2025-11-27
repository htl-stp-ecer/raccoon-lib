//
// Created by tobias on 9/8/25.
//

#pragma once
#include "kinematics/kinematics.hpp"
#include "drive/motor_adapter.hpp"
#include "drive/rate_limiter.hpp"
#include "foundation/config.hpp"
#include "hal/Motor.hpp"
#include <vector>

namespace libstp::drive {
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

        struct MotorControl
        {
            drive::MotorAdapter adapter;
            drive::RateLimiter limiter{0.0};
            double target_w{0.0};
        };

        MotorControl left_motor_;
        MotorControl right_motor_;

        double max_wheel_velocity_{0.0};
        double max_wheel_acceleration_{0.0};

    public:
        DifferentialKinematics(hal::motor::Motor* left_motor,
                               hal::motor::Motor* right_motor,
                               double wheelbase,
                               double wheelRadius,
                               double max_velocity,
                               double max_acceleration);
        ~DifferentialKinematics() override = default;

        void setWheelLimits(double max_velocity, double max_acceleration);

        [[nodiscard]] std::size_t wheelCount() const override;
        MotorCommands applyCommand(const foundation::ChassisCmd& cmd, double dt) override;
        [[nodiscard]] foundation::ChassisState estimateState() const override;
        void hardStop() override;
        [[nodiscard]] bool supportsLateralMotion() const override;
        void resetEncoders() override;

        // Calibration methods
        std::vector<drive::CalibrationResult> calibrateMotors();
        std::vector<drive::CalibrationResult> calibrateMotors(const drive::CalibrationConfig& config);
    };
}
