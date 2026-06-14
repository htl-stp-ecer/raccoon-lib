#pragma once

#include <cstdint>
#include <vector>

#include "autotune/control/control_element.hpp"
#include "autotune/control/control_loop.hpp"
#include "autotune/control/differential_evolution.hpp"

namespace libstp::autotune::control
{

/**
 * @brief Weighted step-response error for controller tuning.
 *        Port of ControlOptimizationError.ts.
 *
 * Per-sample cost = (w - y)^2, plus a penalty proportional to control-output
 * overshoot above stepHeight (weightingCtrlOut) and a penalty proportional to
 * plant-output overshoot above stepHeight + maxOvershot (weightingOvershot).
 */
class ControlOptimizationError
{
public:
    ControlOptimizationError(const ControlLoop& controlLoop,
                             double             stepHeight,
                             double             maxOvershot,
                             double             weightingCtrlOut,
                             double             weightingOvershot);

    [[nodiscard]] double calcError(double w, double y) const;

private:
    const ControlLoop* control_loop_;
    double             step_height_;
    double             max_overshot_;
    double             weighting_ctrl_out_;
    double             weighting_overshot_;
};

/**
 * @brief PID-gain optimization via DE.  Port of
 *        ControllerOptimizationDifferentialEvolution.ts.
 *
 * Simulates the closed loop over a fixed number of time steps (NUM_TIME_STEPS =
 * 5000) for each candidate gain vector and accumulates the weighted step error.
 * Implements IDifferentialEvolutionRating.
 */
class ControllerOptimizationDifferentialEvolution final : public IDifferentialEvolutionRating
{
public:
    ControllerOptimizationDifferentialEvolution(ControlLoop&    controlLoop,
                                                ControlElement& controlOptimization);

    std::vector<double> optimize(double limitMinParams,
                                 double limitMaxParams,
                                 bool   dither,
                                 int    maxNumSteps,
                                 double stepHeight,
                                 double stepTime,
                                 double endTime,
                                 double maxOvershot,
                                 double weightingCtrlOut,
                                 double weightingOvershot,
                                 std::uint32_t seed = 0xC0FFEEu);

    double rateCandidate(const std::vector<double>& paramList) override;

    /// Configure the rating state directly (normally done by optimize()).
    /// Exposed for direct cost-function testing.
    void setStep(double stepHeight,
                 double stepTime,
                 double endTime,
                 double maxOvershot,
                 double weightingCtrlOut,
                 double weightingOvershot);

private:
    static constexpr int kNumTimeSteps = 5000;

    double evaluate(const std::vector<double>& params,
                    double                     stepHeight,
                    double                     stepTime,
                    double                     endTime);

    ControlLoop&    control_loop_;
    ControlElement& control_optimization_;

    double step_height_{0};
    double step_time_{0};
    double end_time_{0};

    bool                     have_error_{false};
    ControlOptimizationError calc_error_;
};

} // namespace libstp::autotune::control
