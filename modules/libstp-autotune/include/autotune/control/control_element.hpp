#pragma once

#include <string>
#include <vector>

// C++ port of the TypeScript control-theory toolkit by Torsten Brischalle
// (MIT, http://www.aaabbb.de). The original lives in
// Control/shared/ControlElement*.ts. Only the discrete-time simulation
// (Initialize/Calculate) and DE-optimization (OptimizeGetNumParameters /
// OptimizeSetParameters) paths are ported here — the GUI-only
// TransferFunction()/TimeFunctionStepResponse() (which depend on the Complex
// class) are intentionally omitted as they are not exercised by parameter
// estimation, controller optimization, or the simulation loop.

namespace libstp::autotune::control
{

/**
 * @brief Abstract base for every control element (P, I, D, PT1, ...).
 *
 * Faithful port of `ControlElement.ts`. Implements the optimization interface
 * (`IControlOptimization`) directly: every element exposes how many tunable
 * parameters it has and how to write a flat parameter vector into its state.
 */
class ControlElement
{
public:
    virtual ~ControlElement() = default;

    /// Short type tag, e.g. "P", "PT1", "ControlLoop".
    [[nodiscard]] virtual std::string getType() const = 0;

    /// Lowest time constant of the element, or -1 if it has none.
    [[nodiscard]] virtual double getLowestTimeConstant() const = 0;

    /// Reset internal state for a simulation run at sample time @p dt.
    virtual void initialize(double dt) = 0;

    /// Advance one sample: feed input @p u, return output.
    virtual double calculate(double u, double dt) = 0;

    /// Number of tunable parameters this element consumes.
    [[nodiscard]] virtual int optimizeGetNumParameters() const = 0;

    /// Load parameters from @p params starting at @p startIdx.
    virtual void optimizeSetParameters(const std::vector<double>& params,
                                       int                        startIdx) = 0;
};

} // namespace libstp::autotune::control
