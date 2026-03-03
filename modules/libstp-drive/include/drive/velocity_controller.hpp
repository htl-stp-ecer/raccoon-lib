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
    /**
     * PID plus feedforward helper used for chassis velocity control.
     *
     * The controller uses measured-derivative filtering and a simple
     * back-calculation style anti-windup term when clamping is active.
     */
    class VelocityController
    {
    public:
        /** Construct a controller with the supplied PID and feedforward terms. */
        VelocityController(foundation::PidGains g, foundation::Feedforward ff);

        /** Replace the PID gains used by subsequent `compute()` calls. */
        void setGains(const foundation::PidGains& g);

        /** Replace the feedforward terms used by subsequent `compute()` calls. */
        void setFF(const foundation::Feedforward& ff);

        const foundation::PidGains& gains() const { return g_; }
        const foundation::Feedforward& ff() const { return ff_; }

        /**
         * Compute the next controller output.
         *
         * @param w_ref Reference velocity.
         * @param a_ref Reference acceleration for the feedforward term.
         * @param w_meas Measured velocity.
         * @param dt Sample time in seconds.
         * @param u_max Symmetric output clamp magnitude.
         * @param out_saturated Optional flag set when clamping occurs.
         */
        double compute(double w_ref, double a_ref, double w_meas, double dt,
                       double u_max, bool* out_saturated = nullptr);

        /** Clear integrator, derivative filter, and measurement history. */
        void reset();

    private:
        foundation::PidGains g_;
        foundation::Feedforward ff_;

        double i_{0.0};
        double w_meas_prev_{0.0};
        double d_filt_{0.0};

        double d_alpha_{0.3};
        double k_aw_{0.4};
    };
}
