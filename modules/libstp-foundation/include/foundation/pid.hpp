#pragma once

#include "foundation/motor.hpp"

namespace libstp::foundation
{
    /// Full PID configuration including anti-windup, derivative filtering, and output limits.
    struct PidConfig
    {
        double kp{1.0};
        double ki{0.0};
        double kd{0.0};
        double integral_max{10.0};
        double integral_deadband{0.01};
        double derivative_lpf_alpha{0.1};
        double output_min{-10.0};
        double output_max{10.0};

        PidConfig() = default;

        PidConfig(double kp, double ki, double kd,
                  double integral_max = 10.0, double integral_deadband = 0.01,
                  double derivative_lpf_alpha = 0.1,
                  double output_min = -10.0, double output_max = 10.0)
            : kp(kp), ki(ki), kd(kd)
            , integral_max(integral_max), integral_deadband(integral_deadband)
            , derivative_lpf_alpha(derivative_lpf_alpha)
            , output_min(output_min), output_max(output_max)
        {}

        // Convenience: construct from simple PidGains (advanced params get defaults).
        PidConfig(const PidGains& g) : kp(g.kp), ki(g.ki), kd(g.kd) {}
    };

    /// Stateful PID controller with integral clamping and a low-pass filtered derivative term.
    class PidController
    {
    public:
        explicit PidController(PidConfig config = {});

        /// Update the controller using `error` for both the proportional and derivative signals.
        double update(double error, double dt);

        /**
         * Update with separate derivative signal (derivative-on-measurement).
         * P and I terms use @p error, D term uses @p deriv_signal.
         */
        double update(double error, double dt, double deriv_signal);

        /// Clear accumulated integral and derivative state.
        void reset();

        /// Replace only the proportional, integral, and derivative gains.
        void setGains(double kp, double ki, double kd);

        [[nodiscard]] double getIntegral() const { return integral_; }
        [[nodiscard]] double getDerivative() const { return filtered_derivative_; }

    private:
        PidConfig cfg_;
        double integral_{0.0};
        double prev_error_{0.0};
        double filtered_derivative_{0.0};
        bool first_update_{true};
    };
}
