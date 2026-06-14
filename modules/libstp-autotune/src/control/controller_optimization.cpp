#include "autotune/control/controller_optimization.hpp"

namespace libstp::autotune::control
{

// ----------------------------------------------------- ControlOptimizationError

ControlOptimizationError::ControlOptimizationError(const ControlLoop& controlLoop,
                                                   double             stepHeight,
                                                   double             maxOvershot,
                                                   double             weightingCtrlOut,
                                                   double             weightingOvershot)
    : control_loop_(&controlLoop),
      step_height_(stepHeight),
      max_overshot_(maxOvershot),
      weighting_ctrl_out_(weightingCtrlOut),
      weighting_overshot_(weightingOvershot)
{
}

double ControlOptimizationError::calcError(double w, double y) const
{
    const double dy = w - y;
    double       e  = dy * dy;

    if (control_loop_->u() > step_height_)
    {
        const double du = control_loop_->u() - step_height_;
        e += du * du * weighting_ctrl_out_;
    }

    if (y > step_height_ + max_overshot_)
    {
        const double dd = y - (step_height_ + max_overshot_);
        e += dd * dd * weighting_overshot_;
    }

    return e;
}

// ----------------------- ControllerOptimizationDifferentialEvolution

ControllerOptimizationDifferentialEvolution::ControllerOptimizationDifferentialEvolution(
    ControlLoop&    controlLoop,
    ControlElement& controlOptimization)
    : control_loop_(controlLoop),
      control_optimization_(controlOptimization),
      calc_error_(controlLoop, 0, 0, 0, 0)
{
}

void ControllerOptimizationDifferentialEvolution::setStep(double stepHeight,
                                                          double stepTime,
                                                          double endTime,
                                                          double maxOvershot,
                                                          double weightingCtrlOut,
                                                          double weightingOvershot)
{
    calc_error_ = ControlOptimizationError(control_loop_, stepHeight, maxOvershot, weightingCtrlOut,
                                           weightingOvershot);
    have_error_  = true;
    step_height_ = stepHeight;
    step_time_   = stepTime;
    end_time_    = endTime;
}

std::vector<double> ControllerOptimizationDifferentialEvolution::optimize(
    double        limitMinParams,
    double        limitMaxParams,
    bool          dither,
    int           maxNumSteps,
    double        stepHeight,
    double        stepTime,
    double        endTime,
    double        maxOvershot,
    double        weightingCtrlOut,
    double        weightingOvershot,
    std::uint32_t seed)
{
    setStep(stepHeight, stepTime, endTime, maxOvershot, weightingCtrlOut, weightingOvershot);

    const int numParams = control_optimization_.optimizeGetNumParameters();

    DifferentialEvolution diffEvo(*this, numParams, dither, seed);

    std::vector<double> initMinParamsArr(static_cast<std::size_t>(numParams), 0.0);
    std::vector<double> initMaxParamsArr(static_cast<std::size_t>(numParams), 1.0);
    std::vector<double> limitMinParamsArr(static_cast<std::size_t>(numParams), limitMinParams);
    std::vector<double> limitMaxParamsArr(static_cast<std::size_t>(numParams), limitMaxParams);

    diffEvo.createCandidates(initMinParamsArr, initMaxParamsArr, limitMinParamsArr,
                             limitMaxParamsArr);

    for (int step = 0; step < maxNumSteps; step++)
        diffEvo.evolutionStep();

    return diffEvo.getBestCandidate();
}

double ControllerOptimizationDifferentialEvolution::rateCandidate(
    const std::vector<double>& paramList)
{
    return evaluate(paramList, step_height_, step_time_, end_time_);
}

double ControllerOptimizationDifferentialEvolution::evaluate(const std::vector<double>& params,
                                                             double                     stepHeight,
                                                             double                     stepTime,
                                                             double                     endTime)
{
    const double dt  = endTime / kNumTimeSteps;
    double       t   = 0;
    double       w   = 0;
    double       err = 0;

    control_optimization_.optimizeSetParameters(params, 0);
    control_loop_.initialize(dt);

    while (t <= endTime)
    {
        if (t >= stepTime)
            w = stepHeight;

        const double y = control_loop_.calculate(w, dt);

        err += calc_error_.calcError(w, y);

        t += dt;
    }

    return err;
}

} // namespace libstp::autotune::control
