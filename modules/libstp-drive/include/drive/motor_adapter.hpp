//
// Created by tobias on 9/8/25.
//

#pragma once
#include "hal/Motor.hpp"
#include "velocity_controller.hpp"

namespace libstp::drive
{
    struct MotorCalibration
    {
        double velocity_to_percent_scale{10.0};
        double percent_to_velocity_scale{0.1};

        VelocityController::Gains pid_gains{1.0, 0.0, 0.0, 1.0};

        double bemf_offset{0.0};
        double max_percent_output{100.0};
    };

    class MotorAdapter
    {
    public:
        explicit MotorAdapter(const hal::motor::Motor& motor, const MotorCalibration& calibration = {});

        void setVelocity(double target_rad_per_s, double dt);

        [[nodiscard]] double getVelocity() const;

        void setPercent(double percent);

        [[nodiscard]] int getRawPercent() const;

        void setCalibration(const MotorCalibration& calibration);
        [[nodiscard]] const MotorCalibration& getCalibration() const;

        void resetController();
        void brake();

        hal::motor::Motor& getMotor();
        [[nodiscard]] const hal::motor::Motor& getMotor() const;

    private:
        hal::motor::Motor motor_;
        MotorCalibration calibration_;
        VelocityController controller_;

        double last_target_velocity_{0.0};
        bool velocity_control_active_{false};
    };
}
