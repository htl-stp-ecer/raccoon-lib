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
        Feedforward ff{};
        PidGains pid{1.0, 0.0, 0.0};


        Deadzone deadzone{true, 10.0, 40.0, 15.0};

        double max_percent_output{100.0};
        double ticks_to_rad{1.0};
        double vel_lpf_alpha{0.2};

        bool invert_meas{false};
        bool invert_cmd{false};
    };


    class MotorAdapter
    {
    public:
        explicit MotorAdapter(hal::motor::Motor* motor, const MotorCalibration& calibration = {});

        void setVelocityWithAccel(double w_ref, double a_ref, double dt, bool* out_saturated);

        void setVelocity(double w_ref, double dt);

        void setPercent(double percent);

        [[nodiscard]] double getVelocity() const;

        [[nodiscard]] int getRawPercent() const;

        void setCalibration(const MotorCalibration& calibration);
        [[nodiscard]] const MotorCalibration& getCalibration() const;

        void resetController();
        void brake();

        hal::motor::Motor& motor();
        [[nodiscard]] const hal::motor::Motor& motor() const;

        void updateEncoderVelocity(double dt);

    private:
        hal::motor::Motor* motor_{nullptr};
        MotorCalibration calibration_;
        VelocityController controller_;

        mutable double w_meas_filt_{0.0};
        long long pos_prev_{0};
        bool pos_prev_init_{false};

        double last_u_cmd_{0.0};
    };
}
