//
// Created by tobias on 9/8/25.
//

#pragma once
#include "kinematics/kinematics.hpp"
#include "drive/motor_adapter.hpp"
#include "drive/rate_limiter.hpp"
#include "hal/Motor.hpp"

namespace libstp::kinematics::mecanum
{
    class MecanumKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_trackWidth;
        double m_wheelRadius;

        struct MotorControl
        {
            drive::MotorAdapter adapter;
            drive::RateLimiter limiter{0.0};
            double target_w{0.0};
        };

        MotorControl front_left_motor_;
        MotorControl front_right_motor_;
        MotorControl back_left_motor_;
        MotorControl back_right_motor_;

        double max_wheel_velocity_{0.0};
        double max_wheel_acceleration_{0.0};

    public:
        MecanumKinematics(hal::motor::Motor* front_left_motor,
                         hal::motor::Motor* front_right_motor,
                         hal::motor::Motor* back_left_motor,
                         hal::motor::Motor* back_right_motor,
                         double wheelbase,
                         double trackWidth,
                         double wheelRadius,
                         double max_velocity,
                         double max_acceleration);
        ~MecanumKinematics() override = default;

        void setWheelLimits(double max_velocity, double max_acceleration);

        [[nodiscard]] std::size_t wheelCount() const override;
        void applyCommand(const foundation::ChassisCmd& cmd, double dt) override;
        [[nodiscard]] foundation::ChassisState estimateState() const override;
        void hardStop() override;
    };
}