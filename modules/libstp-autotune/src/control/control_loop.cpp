#include "autotune/control/control_loop.hpp"

#include <algorithm>

namespace libstp::autotune::control
{

ControlLoop::ControlLoop(ControlElement& controller, ControlElement& controlPath)
    : controller_(controller), control_path_(controlPath)
{
}

double ControlLoop::getLowestTimeConstant() const
{
    const double t1 = controller_.getLowestTimeConstant();
    const double t2 = control_path_.getLowestTimeConstant();

    if (t1 == -1)
        return t2;
    if (t2 == -1)
        return t1;

    return std::min(t1, t2);
}

void ControlLoop::initialize(double dt)
{
    u_         = 0;
    y_n1_      = 0;
    y_min_     = 0;
    y_max_     = 0;
    direction_ = 1;

    controller_.initialize(dt);
    control_path_.initialize(dt);
}

double ControlLoop::calculate(double w, double dt)
{
    const double e = w - y_n1_;
    const double u = controller_.calculate(e, dt);
    const double y = control_path_.calculate(u, dt);

    if ((direction_ == -1) && (y_n1_ < y))
    {
        y_min_     = y_n1_;
        direction_ = 1;
    }

    if ((direction_ == 1) && (y_n1_ > y))
    {
        y_max_     = y_n1_;
        direction_ = -1;
    }

    y_n1_ = y;
    u_    = u;

    return y;
}

int ControlLoop::optimizeGetNumParameters() const
{
    return controller_.optimizeGetNumParameters() + control_path_.optimizeGetNumParameters();
}

void ControlLoop::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    controller_.optimizeSetParameters(params, startIdx);
    control_path_.optimizeSetParameters(params,
                                        startIdx + controller_.optimizeGetNumParameters());
}

} // namespace libstp::autotune::control
