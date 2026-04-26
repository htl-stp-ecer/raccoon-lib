#pragma once

#include "foundation/motor.hpp"

namespace libstp::foundation
{
    /// Full PID configuration including anti-windup, derivative filtering, and output limits.
    ///
    /// Units: error and the controller output are in **whatever units the
    /// caller chose**. PidController is unit-agnostic — pass it errors in
    /// rad/s, m, or normalised speed-scale, and the same units come back as
    /// output. The defaults below are sized for the legacy "drive in
    /// normalised [-1, 1] command space" use case from libstp; new
    /// integrations should override them.
    ///
    /// **Defaults rationale:**
    /// - `kp = 1.0`, `ki = 0`, `kd = 0` — proportional-only baseline; the
    ///   user is expected to tune the gains, not inherit them.
    /// - `integral_max = 10.0` — integrator clamp in *output* units. The
    ///   default assumes output is roughly bounded to ±10 (matches
    ///   `output_min/max` below). If your output range is normalised speed
    ///   ([-1, 1]), set this to ~1.0 instead, or the integrator can take
    ///   seconds to recover from saturation.
    /// - `integral_deadband = 0.01` — error magnitudes below this don't
    ///   accumulate into the integrator. In the legacy "normalised speed"
    ///   convention this is 1% of full output.
    /// - `derivative_lpf_alpha = 0.1` — IIR weight on the new derivative
    ///   sample (0 = use only history, 1 = no filter). 0.1 corresponds to
    ///   a ~10-sample moving-average response, suitable for ~10 ms control
    ///   loops with noisy feedback.
    /// - `output_min/max = ±10` — output saturation in caller units. Override
    ///   when feeding a downstream controller with a different scale.
    struct PidConfig
    {
        double kp{1.0};
        double ki{0.0};
        double kd{0.0};
        double integral_max{10.0};       ///< Integrator clamp, in output units.
        double integral_deadband{0.01};  ///< Error magnitude below which the integrator ignores the sample.
        double derivative_lpf_alpha{0.1};///< IIR weight on new derivative samples; clamped to [0, 1] at runtime.
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
