//
// Created by tobias on 9/8/25.
//

//
// Created by tobias on 9/8/25.
//

#pragma once

namespace libstp::drive
{
    struct PidGains
    {
        double kp{0.0}, ki{0.0}, kd{0.0};
    };

    struct Feedforward
    {
        double kS{0.0};
        double kV{0.0};
        double kA{0.0};
    };

    struct Deadzone
    {
        bool enable{false};
        double zero_window_percent{0.0};
        double start_percent{0.0};
        double release_percent{0.0};
    };

    class VelocityController
    {
    public:
        VelocityController(PidGains g, Feedforward ff, Deadzone dz);

        void setGains(const PidGains& g);
        void setFF(const Feedforward& ff);
        void setDeadzone(const Deadzone& dz);

        const PidGains& gains() const { return g_; }
        const Feedforward& ff() const { return ff_; }
        const Deadzone& deadzone() const { return dz_; }

        double compute(double w_ref, double a_ref, double w_meas, double dt,
                       double u_max, bool* out_saturated = nullptr);

        void reset();

    private:
        PidGains g_;
        Feedforward ff_;
        Deadzone dz_;

        double i_{0.0};
        double w_meas_prev_{0.0};
        double d_filt_{0.0};

        double d_alpha_{0.15};
        double k_aw_{0.4};

        double mapDutyWithDeadzone(double u_raw, double u_max) const;
    };
}
