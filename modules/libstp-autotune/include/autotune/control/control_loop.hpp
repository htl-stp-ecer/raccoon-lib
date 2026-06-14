#pragma once

#include <string>
#include <vector>

#include "autotune/control/control_element.hpp"

namespace libstp::autotune::control
{

/**
 * @brief Closed unity-feedback loop: controller in series with plant.
 *        Port of ControlLoop.ts.
 *
 * Each step computes error e = w - y_prev, drives it through the controller to
 * get the manipulated variable u, then through the plant to get the output y.
 * Tracks the most recent local min/max of y (yMin/yMax) for amplitude/overshoot
 * analysis. Does NOT own the controller/plant — they are borrowed references,
 * matching the TypeScript constructor semantics.
 */
class ControlLoop final : public ControlElement
{
public:
    ControlLoop(ControlElement& controller, ControlElement& controlPath);

    [[nodiscard]] double u() const { return u_; }
    [[nodiscard]] double yMin() const { return y_min_; }
    [[nodiscard]] double yMax() const { return y_max_; }
    [[nodiscard]] double amplitude() const { return (y_max_ - y_min_) / 2; }

    [[nodiscard]] std::string getType() const override { return "ControlLoop"; }
    [[nodiscard]] double      getLowestTimeConstant() const override;
    void                      initialize(double dt) override;
    double                    calculate(double w, double dt) override;
    [[nodiscard]] int         optimizeGetNumParameters() const override;
    void optimizeSetParameters(const std::vector<double>& params, int startIdx) override;

private:
    ControlElement& controller_;
    ControlElement& control_path_;

    double u_{0};
    double y_n1_{0};
    double y_min_{0};
    double y_max_{0};
    int    direction_{1};
};

} // namespace libstp::autotune::control
