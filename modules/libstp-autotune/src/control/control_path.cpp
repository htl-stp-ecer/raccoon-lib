#include "autotune/control/control_path.hpp"

#include <utility>

namespace libstp::autotune::control
{

void ControlPath::add(std::unique_ptr<ControlElement> element)
{
    elements_.push_back(std::move(element));
}

double ControlPath::getLowestTimeConstant() const
{
    double t = -1;

    for (const auto& el : elements_)
    {
        const double tt = el->getLowestTimeConstant();

        if (t == -1)
        {
            t = tt;
        }
        else
        {
            if ((tt != -1) && (tt < t))
                t = tt;
        }
    }

    return t;
}

void ControlPath::initialize(double dt)
{
    for (const auto& el : elements_)
        el->initialize(dt);
}

double ControlPath::calculate(double u, double dt)
{
    double y = u;

    for (const auto& el : elements_)
        y = el->calculate(y, dt);

    return y;
}

int ControlPath::optimizeGetNumParameters() const
{
    int numParams = 0;

    for (const auto& el : elements_)
        numParams += el->optimizeGetNumParameters();

    return numParams;
}

void ControlPath::optimizeSetParameters(const std::vector<double>& params, int startIdx)
{
    int paramIdx = startIdx;

    for (const auto& el : elements_)
    {
        el->optimizeSetParameters(params, paramIdx);
        paramIdx += el->optimizeGetNumParameters();
    }
}

} // namespace libstp::autotune::control
