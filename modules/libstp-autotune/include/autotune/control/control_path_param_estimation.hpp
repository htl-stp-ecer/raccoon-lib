#pragma once

#include <utility>
#include <vector>

#include "autotune/control/control_path.hpp"
#include "autotune/control/differential_evolution.hpp"

namespace libstp::autotune::control
{

/**
 * @brief Plant system identification via DE.  Port of
 *        ControlPathParamEstimation.ts.
 *
 * Given measured step-response data (time, value) pairs, finds the control-path
 * parameters that minimise the squared error between the simulated and measured
 * output. The sample time dt is derived as the mean spacing of the measurement
 * timestamps, exactly as in the TypeScript.
 *
 * Implements IDifferentialEvolutionRating so the DE optimizer can rate
 * candidate parameter vectors.
 */
class ControlPathParamEstimation final : public IDifferentialEvolutionRating
{
public:
    using MeasPoint = std::pair<double, double>;

    ControlPathParamEstimation(ControlPath&                  controlPath,
                               const std::vector<MeasPoint>& measData);

    /**
     * @brief Run DE to fit the control path to the measured data.
     * @return best-fit parameter vector.
     */
    std::vector<double> optimize(double limitMinParams,
                                 double limitMaxParams,
                                 bool   dither,
                                 int    maxNumSteps,
                                 double stepHeight,
                                 double stepTime,
                                 std::uint32_t seed = 0xC0FFEEu);

    /// DE cost callback: squared error of the simulated vs. measured response.
    double rateCandidate(const std::vector<double>& paramList) override;

    /// Mean sample time derived from the measurement timestamps.
    [[nodiscard]] double dt() const { return dt_; }

    /// Set the step parameters that rateCandidate() uses (normally set by
    /// optimize()). Exposed for direct cost-function testing.
    void setStep(double stepHeight, double stepTime)
    {
        step_height_ = stepHeight;
        step_time_   = stepTime;
    }

private:
    double evaluate(const std::vector<double>& params,
                    double                     stepHeight,
                    double                     stepTime);

    ControlPath&           control_path_;
    std::vector<MeasPoint> meas_data_;
    double                 dt_{0};
    double                 step_height_{0};
    double                 step_time_{0};
};

} // namespace libstp::autotune::control
