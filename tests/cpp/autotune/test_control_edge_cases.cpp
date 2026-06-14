// Edge-case tests for the ported control toolkit: degenerate parameters,
// clamping bounds, single-candidate DE, single-step loops, and constructor
// validation that mirrors the TypeScript `throw new Error(...)` guards.
#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include "autotune/control/control_elements.hpp"
#include "autotune/control/control_loop.hpp"
#include "autotune/control/control_path.hpp"
#include "autotune/control/differential_evolution.hpp"

using namespace libstp::autotune::control;

namespace
{
class SphereRating final : public IDifferentialEvolutionRating
{
public:
    double rateCandidate(const std::vector<double>& p) override
    {
        double s = 0;
        for (double x : p) s += x * x;
        return s;
    }
};
} // namespace

TEST(ControlEdgeCases, NegativeTimeConstantThrows)
{
    EXPECT_THROW(ControlElementI(-1.0), std::invalid_argument);
    EXPECT_THROW(ControlElementPT1(1.0, -1.0), std::invalid_argument);
    EXPECT_THROW(ControlElementPT2(1.0, -1.0, 0.5), std::invalid_argument);
    EXPECT_THROW(ControlElementPT2(1.0, 1.0, -0.5), std::invalid_argument);
    EXPECT_THROW(ControlElementPD1(1.0, -1.0), std::invalid_argument);
    EXPECT_THROW(ControlElementDeadTime(-1.0), std::invalid_argument);
}

TEST(ControlEdgeCases, IElementClampsTinyTimeConstant)
{
    // optimizeSetParameters clamps T to 1e-15 to avoid divide-by-zero blowup.
    ControlElementI el(1.0);
    el.optimizeSetParameters({0.0}, 0);
    el.initialize(0.1);
    const double y = el.calculate(1.0, 0.1);
    // y = dt * u_sum / T = 0.1 * 1 / 1e-15 = 1e14.
    EXPECT_NEAR(y, 1e14, 1.0);
}

TEST(ControlEdgeCases, PT1ClampsNegativeTimeConstantToZero)
{
    ControlElementPT1 el(2.0, 0.5);
    el.optimizeSetParameters({3.0, -1.0}, 0);
    el.initialize(0.1);
    // With T clamped to 0: y = (0 + dt*u*K)/(0+dt) = u*K = 1*3 = 3.
    EXPECT_DOUBLE_EQ(el.calculate(1.0, 0.1), 3.0);
}

TEST(ControlEdgeCases, EmptyControlPath)
{
    ControlPath path;
    EXPECT_EQ(path.optimizeGetNumParameters(), 0);
    EXPECT_DOUBLE_EQ(path.getLowestTimeConstant(), -1);
    path.initialize(0.1);
    // Empty path is a pass-through: y = u.
    EXPECT_DOUBLE_EQ(path.calculate(0.7, 0.1), 0.7);
}

TEST(ControlEdgeCases, ZeroParamsCandidate)
{
    // P element with parameter set to zero -> zero output.
    ControlElementP el(5.0);
    el.optimizeSetParameters({0.0}, 0);
    EXPECT_DOUBLE_EQ(el.calculate(123.0, 0.1), 0.0);
}

TEST(ControlEdgeCases, SingleStepLoop)
{
    ControlElementP   controller(1.0);
    ControlElementPT1 plant(1.0, 0.5);
    ControlLoop       loop(controller, plant);
    loop.initialize(0.1);
    // One step from rest: e = w - 0 = 1, u = 1, y = (0*T + dt*u*K)/(T+dt).
    const double y = loop.calculate(1.0, 0.1);
    EXPECT_NEAR(y, (0.1 * 1.0 * 1.0) / (0.5 + 0.1), 1e-12);
    EXPECT_DOUBLE_EQ(loop.u(), 1.0);
}

TEST(ControlEdgeCases, MinEqualsMaxClampsAllParams)
{
    // When init/limit min==max, every candidate parameter is pinned to that
    // value (Create: min + rand*(max-min) = min), so DE cannot move and the
    // best candidate equals the pinned point.
    SphereRating          r;
    DifferentialEvolution de(r, 2, false, 7);
    de.createCandidates({2.0, 2.0}, {2.0, 2.0}, {2.0, 2.0}, {2.0, 2.0});
    auto best = de.getBestCandidate();
    EXPECT_DOUBLE_EQ(best[0], 2.0);
    EXPECT_DOUBLE_EQ(best[1], 2.0);
    de.evolutionStep();
    best = de.getBestCandidate();
    EXPECT_DOUBLE_EQ(best[0], 2.0);
    EXPECT_DOUBLE_EQ(best[1], 2.0);
}

TEST(ControlEdgeCases, SingleParamDESmallPopulation)
{
    // numParams=1 -> population size = 1 * 12 = 12. getThreeRandomCandidates
    // requires at least 4 distinct indices, which 12 satisfies. Verify a run
    // does not hang or crash and produces a finite result.
    SphereRating r;
    DifferentialEvolution de(r, 1, false, 3);
    de.createCandidates({-1.0}, {1.0}, {-5.0}, {5.0});
    for (int i = 0; i < 10; i++) de.evolutionStep();
    auto best = de.getBestCandidate();
    ASSERT_EQ(best.size(), 1u);
    EXPECT_TRUE(std::isfinite(best[0]));
}

TEST(ControlEdgeCases, MutationClampsToLimits)
{
    // Tight limit window forces every mutated parameter into [limMin, limMax].
    SphereRating          r;
    DifferentialEvolution de(r, 2, false, 11);
    de.createCandidates({-0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}, {-0.5, 0.5});
    de.evolutionStep();
    auto best = de.getBestCandidate();
    for (double x : best)
    {
        EXPECT_GE(x, -0.5);
        EXPECT_LE(x, 0.5);
    }
}

TEST(ControlEdgeCases, DeadTimeZeroDelayInitializes)
{
    // T=0 with finite dt: cnt = ceil((0/0.1 + 3)*2) = 6 slots; first sample
    // already past the delay so it reads from the (zero) buffer.
    ControlElementDeadTime el(0.0);
    el.initialize(0.1);
    // First output reads buffer slot 0 (still 0 because write happened to
    // slot 0 then read slot 0 -> equals the just-written value).
    EXPECT_DOUBLE_EQ(el.calculate(0.9, 0.1), 0.9);
}
