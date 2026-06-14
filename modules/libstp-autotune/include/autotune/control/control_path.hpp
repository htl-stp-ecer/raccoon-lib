#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autotune/control/control_element.hpp"

namespace libstp::autotune::control
{

/**
 * @brief Series chain of control elements.  Port of ControlPath.ts.
 *
 * Output of element i feeds element i+1. Owns its elements via unique_ptr so
 * callers can build a path with `add(std::make_unique<...>())`.
 */
class ControlPath final : public ControlElement
{
public:
    ControlPath() = default;

    /// Append an element to the chain (takes ownership).
    void add(std::unique_ptr<ControlElement> element);

    [[nodiscard]] std::string getType() const override { return "ControlPath"; }
    [[nodiscard]] double      getLowestTimeConstant() const override;
    void                      initialize(double dt) override;
    double                    calculate(double u, double dt) override;
    [[nodiscard]] int         optimizeGetNumParameters() const override;
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    std::vector<std::unique_ptr<ControlElement>> elements_;
};

} // namespace libstp::autotune::control
