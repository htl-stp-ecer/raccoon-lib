#pragma once

#include <string>
#include <vector>

#include "autotune/control/control_element.hpp"

// Faithful ports of the individual control elements
// (Control/shared/ControlElement{P,I,D,PT1,PT2,PD1,DeadTime}.ts). Discrete-time
// difference equations match the TypeScript Calculate() implementations
// exactly so the simulation produces bit-identical numerical results.

namespace libstp::autotune::control
{

/// Proportional element: y = K * u.  Port of ControlElementP.ts.
class ControlElementP final : public ControlElement
{
public:
    explicit ControlElementP(double k) : k_(k) {}

    [[nodiscard]] std::string getType() const override { return "P"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return -1; }
    void                 initialize(double /*dt*/) override {}
    double               calculate(double u, double /*dt*/) override { return k_ * u; }
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 1; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override
    {
        k_ = params[static_cast<std::size_t>(startIdx + 0)];
    }

private:
    double k_;
};

/// Integrator: y += dt * sum(u) / T.  Port of ControlElementI.ts.
class ControlElementI final : public ControlElement
{
public:
    explicit ControlElementI(double t);

    [[nodiscard]] std::string getType() const override { return "I"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return t_; }
    void                 initialize(double /*dt*/) override { u_sum_ = 0; }
    double               calculate(double u, double dt) override;
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 1; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    double t_;
    double u_sum_{0};
};

/// Differentiator: y = K * (u - u_prev) / dt.  Port of ControlElementD.ts.
class ControlElementD final : public ControlElement
{
public:
    explicit ControlElementD(double k) : k_(k) {}

    [[nodiscard]] std::string getType() const override { return "D"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return -1; }
    void                 initialize(double /*dt*/) override { u_n1_ = 0; }
    double               calculate(double u, double dt) override;
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 1; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override
    {
        k_ = params[static_cast<std::size_t>(startIdx + 0)];
    }

private:
    double k_;
    double u_n1_{0};
};

/// First-order lag.  Port of ControlElementPT1.ts.
class ControlElementPT1 final : public ControlElement
{
public:
    ControlElementPT1(double k, double t);

    [[nodiscard]] std::string getType() const override { return "PT1"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return t_; }
    void                 initialize(double /*dt*/) override { y_n1_ = 0; }
    double               calculate(double u, double dt) override;
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 2; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    double k_;
    double t_;
    double y_n1_{0};
};

/// Second-order lag.  Port of ControlElementPT2.ts.
class ControlElementPT2 final : public ControlElement
{
public:
    ControlElementPT2(double k, double t, double d);

    [[nodiscard]] std::string getType() const override { return "PT2"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return t_; }
    void                 initialize(double /*dt*/) override { y_n1_ = 0; y_n2_ = 0; }
    double               calculate(double u, double dt) override;
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 3; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    double k_;
    double t_;
    double d_;
    double y_n1_{0};
    double y_n2_{0};
};

/// Proportional-derivative (first order).  Port of ControlElementPD1.ts.
class ControlElementPD1 final : public ControlElement
{
public:
    ControlElementPD1(double k, double t);

    [[nodiscard]] std::string getType() const override { return "PD1"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return t_; }
    void                 initialize(double /*dt*/) override { u_n1_ = 0; }
    double               calculate(double u, double dt) override;
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 2; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    double k_;
    double t_;
    double u_n1_{0};
};

/// Full dt-EXPLICIT PID controller.
///
/// Mirrors the dt-explicit STM32 firmware `pid.c`:
///     iErr += e * dt;
///     dErr  = (e - prevErr) / dt;
///     u     = kP*e + kI*iErr + kD*dErr;
/// so kI/kD are physical per-second gains and DE-optimized gains transfer
/// directly to the firmware. This element is NOT part of Brischalle's TS
/// toolkit — it intentionally diverges from the toolkit's separate
/// ControlElementI/D (which use the toolkit's own discrete forms) to match the
/// firmware exactly.
class ControlElementPID final : public ControlElement
{
public:
    // out_max / i_max default to "unbounded" (huge) so the element is a pure
    // parallel PID unless saturation is requested. When tuning the firmware
    // velocity loop, set both to MOTOR_MAX_DUTYCYCLE so the simulated loop
    // saturates exactly like the STM32 pid.c — otherwise an unbounded sim
    // output produces fictional overshoot and DE picks uselessly small gains.
    ControlElementPID(double kp, double ki, double kd,
                      double out_max = 1e30, double i_max = 1e30)
        : kp_(kp), ki_(ki), kd_(kd), out_max_(out_max), i_max_(i_max)
    {
    }

    [[nodiscard]] std::string getType() const override { return "PID"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return -1; }
    void                 initialize(double /*dt*/) override
    {
        iErr_    = 0;
        prevErr_ = 0;
    }
    double calculate(double e, double dt) override
    {
        double d = dt;
        if (d <= 0.0)
            d = 1e-9;
        iErr_ += e * d;
        const double dErr = (e - prevErr_) / d;
        prevErr_          = e;

        // Integral anti-windup: clamp the I-term and back-calculate iErr,
        // matching stm32 pid.c.
        double iTerm = ki_ * iErr_;
        if (ki_ > 0.0)
        {
            if (iTerm > i_max_)
            {
                iTerm = i_max_;
                iErr_ = i_max_ / ki_;
            }
            else if (iTerm < -i_max_)
            {
                iTerm = -i_max_;
                iErr_ = -i_max_ / ki_;
            }
        }

        // Output saturation, matching stm32 pid.c clamp to +/-MOTOR_MAX_DUTYCYCLE.
        double cmd = kp_ * e + iTerm + kd_ * dErr;
        if (cmd > out_max_)
            cmd = out_max_;
        else if (cmd < -out_max_)
            cmd = -out_max_;
        return cmd;
    }
    [[nodiscard]] int optimizeGetNumParameters() const override { return 3; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override
    {
        kp_ = params[static_cast<std::size_t>(startIdx + 0)];
        ki_ = params[static_cast<std::size_t>(startIdx + 1)];
        kd_ = params[static_cast<std::size_t>(startIdx + 2)];
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double out_max_;
    double i_max_;
    double iErr_{0};
    double prevErr_{0};
};

/// Pure transport delay.  Port of ControlElementDeadTime.ts.
class ControlElementDeadTime final : public ControlElement
{
public:
    explicit ControlElementDeadTime(double t);

    [[nodiscard]] std::string getType() const override { return "DeadTime"; }
    [[nodiscard]] double getLowestTimeConstant() const override { return t_; }
    void                 initialize(double dt) override;
    double               calculate(double u, double dt) override;
    [[nodiscard]] int    optimizeGetNumParameters() const override { return 1; }
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    double              t_;
    double              y_{0};
    std::vector<double> u_arr_;
    std::size_t         read_idx_{0};
    std::size_t         write_idx_{0};
    double              tt_{0};
};

} // namespace libstp::autotune::control
