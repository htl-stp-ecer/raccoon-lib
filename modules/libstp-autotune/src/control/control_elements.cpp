#include "autotune/control/control_elements.hpp"

#include <cmath>
#include <stdexcept>

namespace libstp::autotune::control
{

// ---------------------------------------------------------------- I element

ControlElementI::ControlElementI(double t) : t_(t)
{
    if (t < 0)
        throw std::invalid_argument("T must be greater then zero!");
}

double ControlElementI::calculate(double u, double dt)
{
    u_sum_ += u;
    return dt * u_sum_ / t_;
}

void ControlElementI::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    t_ = params[static_cast<std::size_t>(startIdx + 0)];
    if (t_ < 1E-15)
        t_ = 1E-15;
}

// ---------------------------------------------------------------- D element

double ControlElementD::calculate(double u, double dt)
{
    const double y = k_ * (u - u_n1_) / dt;
    u_n1_          = u;
    return y;
}

// -------------------------------------------------------------- PT1 element

ControlElementPT1::ControlElementPT1(double k, double t) : k_(k), t_(t)
{
    if (t < 0)
        throw std::invalid_argument("T must be greater then zero!");
}

double ControlElementPT1::calculate(double u, double dt)
{
    const double k    = k_;
    const double t    = t_;
    const double y_n1 = y_n1_;

    const double y = (y_n1 * t + dt * u * k) / (t + dt);

    y_n1_ = y;
    return y;
}

void ControlElementPT1::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    k_ = params[static_cast<std::size_t>(startIdx + 0)];
    t_ = params[static_cast<std::size_t>(startIdx + 1)];
    if (t_ < 0)
        t_ = 0;
}

// -------------------------------------------------------------- PT2 element

ControlElementPT2::ControlElementPT2(double k, double t, double d) : k_(k), t_(t), d_(d)
{
    if (t < 0)
        throw std::invalid_argument("T must be greater then zero!");
    if (d < 0)
        throw std::invalid_argument("D must be greater then zero!");
}

double ControlElementPT2::calculate(double u, double dt)
{
    const double k    = k_;
    const double t    = t_;
    const double d    = d_;
    const double y_n2 = y_n2_;
    const double y_n1 = y_n1_;

    const double y = ((2 * y_n1 - y_n2) * t * t + 2 * dt * y_n1 * d * t + dt * dt * u * k) /
                     (t * t + 2 * dt * d * t + dt * dt);

    y_n2_ = y_n1_;
    y_n1_ = y;
    return y;
}

void ControlElementPT2::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    k_ = params[static_cast<std::size_t>(startIdx + 0)];
    t_ = params[static_cast<std::size_t>(startIdx + 1)];
    d_ = params[static_cast<std::size_t>(startIdx + 2)];
    if (t_ < 0)
        t_ = 0;
    if (d_ < 0)
        d_ = 0;
}

// -------------------------------------------------------------- PD1 element

ControlElementPD1::ControlElementPD1(double k, double t) : k_(k), t_(t)
{
    if (t < 0)
        throw std::invalid_argument("T must be greater then zero!");
}

double ControlElementPD1::calculate(double u, double dt)
{
    const double y = k_ * (u + (u - u_n1_) * t_ / dt);
    u_n1_          = u;
    return y;
}

void ControlElementPD1::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    k_ = params[static_cast<std::size_t>(startIdx + 0)];
    t_ = params[static_cast<std::size_t>(startIdx + 1)];
    if (t_ < 0)
        t_ = 0;
}

// --------------------------------------------------------- DeadTime element

ControlElementDeadTime::ControlElementDeadTime(double t) : t_(t)
{
    if (t < 0)
        throw std::invalid_argument("T must be greater then zero!");
}

void ControlElementDeadTime::initialize(double dt)
{
    // Matches the TS: cnt = ceil((T/dt + 3) * 2). With dt == 0 the original
    // divides by zero -> Infinity -> ceil(Infinity) -> Infinity, then sets an
    // array length of Infinity which throws in JS. Our tests never call
    // DeadTime with dt == 0, so we follow the finite-dt branch faithfully.
    const auto cnt = static_cast<std::size_t>(std::ceil((t_ / dt + 3) * 2));

    u_arr_.assign(cnt, 0.0);
    read_idx_  = 0;
    write_idx_ = 0;
    tt_        = 0;
    y_         = 0;
}

double ControlElementDeadTime::calculate(double u, double dt)
{
    u_arr_[write_idx_] = u;

    write_idx_++;
    if (write_idx_ >= u_arr_.size())
        write_idx_ = 0;

    if (tt_ < t_)
    {
        tt_ += dt;
    }
    else
    {
        y_ = u_arr_[read_idx_];

        read_idx_++;
        if (read_idx_ >= u_arr_.size())
            read_idx_ = 0;
    }

    return y_;
}

void ControlElementDeadTime::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    t_ = params[static_cast<std::size_t>(startIdx + 0)];
    if (t_ < 0)
        t_ = 0;
}

} // namespace libstp::autotune::control
