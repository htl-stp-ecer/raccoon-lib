//
// Created by tobias on 9/8/25.
//

//
// Created by tobias on 9/8/25.
//

#pragma once
#include "foundation/motor.hpp"

namespace libstp::drive
{
    class VelocityController
    {
    public:
        VelocityController(foundation::PidGains g, foundation::Feedforward ff, foundation::Deadzone dz);

        void setGains(const foundation::PidGains& g);
        void setFF(const foundation::Feedforward& ff);
        void setDeadzone(const foundation::Deadzone& dz);

        const foundation::PidGains& gains() const { return g_; }
        const foundation::Feedforward& ff() const { return ff_; }
        const foundation::Deadzone& deadzone() const { return dz_; }

        double compute(double w_ref, double a_ref, double w_meas, double dt,
                       double u_max, bool* out_saturated = nullptr);

        void reset();

    private:
        foundation::PidGains g_;
        foundation::Feedforward ff_;
        foundation::Deadzone dz_;

        double i_{0.0};
        double w_meas_prev_{0.0};
        double d_filt_{0.0};

        double d_alpha_{0.15};
        double k_aw_{0.4};

        double mapDutyWithDeadzone(double u_raw, double u_max) const;
    };
}
