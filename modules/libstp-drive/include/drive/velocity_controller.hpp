//
// Created by tobias on 9/8/25.
//

#pragma once
#include <algorithm>
#include <cmath>

namespace libstp::drive
{
    class VelocityController
    {
    public:
        struct Gains
        {
            double kp{0}, ki{0}, kd{0}, ff{1.0};
        };

        explicit VelocityController(Gains g);

        void setGains(const Gains& g);
        [[nodiscard]] const Gains& gains() const;

        double compute(double target_w, double meas_w, double dt);

        void reset();

    private:
        Gains g_;
        double i_{0.0};
        double pe_{0.0};
    };
}
