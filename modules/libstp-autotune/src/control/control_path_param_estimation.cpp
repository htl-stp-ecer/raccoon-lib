#include "autotune/control/control_path_param_estimation.hpp"

namespace libstp::autotune::control
{

ControlPathParamEstimation::ControlPathParamEstimation(ControlPath&                  controlPath,
                                                       const std::vector<MeasPoint>& measData)
    : control_path_(controlPath), meas_data_(measData)
{
    dt_ = 0;

    for (std::size_t idx = 1; idx < meas_data_.size(); idx++)
        dt_ += meas_data_[idx].first - meas_data_[idx - 1].first;

    dt_ /= static_cast<double>(meas_data_.size() - 1);
}

std::vector<double> ControlPathParamEstimation::optimize(double        limitMinParams,
                                                         double        limitMaxParams,
                                                         bool          dither,
                                                         int           maxNumSteps,
                                                         double        stepHeight,
                                                         double        stepTime,
                                                         std::uint32_t seed)
{
    step_height_ = stepHeight;
    step_time_   = stepTime;

    const int numParams = control_path_.optimizeGetNumParameters();

    std::vector<double> initMinParamsArr(static_cast<std::size_t>(numParams), 0.0);
    std::vector<double> initMaxParamsArr(static_cast<std::size_t>(numParams), 1.0);
    std::vector<double> limitMinParamsArr(static_cast<std::size_t>(numParams), limitMinParams);
    std::vector<double> limitMaxParamsArr(static_cast<std::size_t>(numParams), limitMaxParams);

    DifferentialEvolution diffEvo(*this, numParams, dither, seed);
    diffEvo.createCandidates(initMinParamsArr, initMaxParamsArr, limitMinParamsArr,
                             limitMaxParamsArr);

    for (int step = 0; step < maxNumSteps; step++)
        diffEvo.evolutionStep();

    return diffEvo.getBestCandidate();
}

double ControlPathParamEstimation::rateCandidate(const std::vector<double>& paramList)
{
    return evaluate(paramList, step_height_, step_time_);
}

double ControlPathParamEstimation::evaluate(const std::vector<double>& params,
                                            double                     stepHeight,
                                            double                     stepTime)
{
    const double dt  = dt_;
    double       t   = 0;
    double       w   = 0;
    double       err = 0;

    control_path_.optimizeSetParameters(params, 0);
    control_path_.initialize(dt);

    for (std::size_t idx = 0; idx < meas_data_.size(); idx++)
    {
        if (t >= stepTime)
            w = stepHeight;

        const double y  = control_path_.calculate(w, dt);
        const double dy = y - meas_data_[idx].second;

        err += dy * dy;

        t += dt;
    }

    return err;
}

} // namespace libstp::autotune::control
