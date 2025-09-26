//
// Created by tobias on 9/8/25.
//

#pragma once
#include "hal/Motor.hpp"
#include "velocity_controller.hpp"

namespace libstp::drive
{
    class MotorAdapter
    {
    public:
        explicit MotorAdapter(hal::motor::Motor* motor);

        void setVelocityWithAccel(double w_ref, double a_ref, double dt, bool* out_saturated);

        void setVelocity(double w_ref, double dt);

        void setPercent(double percent);

        [[nodiscard]] double getVelocity() const;

        [[nodiscard]] int getRawPercent() const;

        void resetController();
        void brake();

        hal::motor::Motor& motor();
        [[nodiscard]] const hal::motor::Motor& motor() const;

        void updateEncoderVelocity(double dt);

    private:
        hal::motor::Motor* motor_{nullptr};
        VelocityController controller_;

        mutable double w_meas_filt_{0.0};
        long long pos_prev_{0};
        bool pos_prev_init_{false};

        double last_u_cmd_{0.0};
    };
}
