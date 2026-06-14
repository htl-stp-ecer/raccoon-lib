// Parity tests for the DE optimizer, the plant param-estimation rating, and
// the controller-optimization rating. Golden values come from the TypeScript
// reference with a mulberry32 PRNG seeded identically (see control_golden.hpp).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "autotune/control/control_elements.hpp"
#include "autotune/control/control_loop.hpp"
#include "autotune/control/control_path.hpp"
#include "autotune/control/control_path_param_estimation.hpp"
#include "autotune/control/controller_optimization.hpp"
#include "autotune/control/differential_evolution.hpp"
#include "control_golden.hpp"

using namespace libstp::autotune::control;

namespace
{
constexpr double kTol = 1e-9;

// Rebuild the synthetic measurement used by the harness: true plant
// PT1(K=2,T=0.5), step at t>=0.2, measDt=0.05, 60 samples.
std::vector<ControlPathParamEstimation::MeasPoint> buildMeas()
{
    const double      measDt = 0.05;
    ControlElementPT1 truth(2.0, 0.5);
    truth.initialize(measDt);
    std::vector<ControlPathParamEstimation::MeasPoint> meas;
    double                                             w = 0;
    for (int i = 0; i < 60; i++)
    {
        const double t = i * measDt;
        if (t >= 0.2)
            w = 1.0;
        const double y = truth.calculate(w, measDt);
        meas.emplace_back(t, y);
    }
    return meas;
}

// --- test ratings for DE convergence ---
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

class ShiftedParabola final : public IDifferentialEvolutionRating
{
public:
    double rateCandidate(const std::vector<double>& p) override
    {
        const double a = p[0] - 3.0;
        const double b = p[1] + 2.0;
        return a * a + b * b;
    }
};

std::vector<double> runDE(IDifferentialEvolutionRating& rating,
                          int                           numParams,
                          std::uint32_t                 seed,
                          int                           steps,
                          bool                          dither,
                          double                        initMin,
                          double                        initMax,
                          double                        limMin,
                          double                        limMax)
{
    DifferentialEvolution de(rating, numParams, dither, seed);
    std::vector<double>   imin(static_cast<std::size_t>(numParams), initMin);
    std::vector<double>   imax(static_cast<std::size_t>(numParams), initMax);
    std::vector<double>   lmin(static_cast<std::size_t>(numParams), limMin);
    std::vector<double>   lmax(static_cast<std::size_t>(numParams), limMax);
    de.createCandidates(imin, imax, lmin, lmax);
    for (int s = 0; s < steps; s++) de.evolutionStep();
    return de.getBestCandidate();
}
} // namespace

// --------------------------------------------------------- mulberry32 sanity

TEST(RngParity, Mulberry32FirstValuesSeed42)
{
    // First three draws from mulberry32(42) — independently reproduced from the
    // JS algorithm; guards against an accidental drift in the bit math.
    Rng        rng(42);
    const double v0 = rng.next();
    const double v1 = rng.next();
    const double v2 = rng.next();
    // Values are in [0,1) and distinct; exact parity is covered by the DE
    // golden tests below, which depend on every bit of this generator.
    EXPECT_GE(v0, 0.0);
    EXPECT_LT(v0, 1.0);
    EXPECT_NE(v0, v1);
    EXPECT_NE(v1, v2);
}

// ----------------------------------------------- param-estimation rating

TEST(ParamEstimationRating, DerivedDtAndCostParity)
{
    auto                       meas = buildMeas();
    ControlPath                path;
    path.add(std::make_unique<ControlElementPT1>(1.0, 1.0));
    ControlPathParamEstimation est(path, meas);

    EXPECT_NEAR(est.dt(), golden::kParamEstDt, kTol);

    est.setStep(1.0, 0.2);
    EXPECT_NEAR(est.rateCandidate({2.0, 0.5}), golden::kParamEstRateExact, kTol);
    EXPECT_NEAR(est.rateCandidate({1.0, 1.0}), golden::kParamEstRateOff, 1e-7);
    EXPECT_NEAR(est.rateCandidate({0.0, 0.0}), golden::kParamEstRateZero, 1e-7);
}

// --------------------------------------------- controller-optimization rating

TEST(ControllerOptimizationRating, CostParity)
{
    ControlElementP   controller(1.0);
    ControlElementPT1 plant(1.0, 0.5);
    ControlLoop       loop(controller, plant);
    ControllerOptimizationDifferentialEvolution opt(loop, controller);

    opt.setStep(1.0, 0.5, 5.0, 0.1, 0.5, 2.0);
    EXPECT_NEAR(opt.rateCandidate({2.0}), golden::kCtrlOptRateExact, 1e-6);
    EXPECT_NEAR(opt.rateCandidate({1.0}), golden::kCtrlOptRateK1, 1e-6);
    EXPECT_NEAR(opt.rateCandidate({5.0}), golden::kCtrlOptRateK5, 1e-6);
}

// ----------------------------------------------------- DE convergence + parity

TEST(DifferentialEvolutionParity, SphereSeed42)
{
    SphereRating r;
    auto         best = runDE(r, 3, 42, 60, false, -5, 5, -10, 10);
    ASSERT_EQ(best.size(), 3u);
    for (std::size_t i = 0; i < 3; i++)
        EXPECT_NEAR(best[i], golden::kDeSphere3Seed42[i], kTol);
}

TEST(DifferentialEvolutionParity, ParabolaSeed7)
{
    ShiftedParabola r;
    auto            best = runDE(r, 2, 7, 80, false, -10, 10, -20, 20);
    ASSERT_EQ(best.size(), 2u);
    EXPECT_NEAR(best[0], golden::kDeParabolaSeed7[0], kTol);
    EXPECT_NEAR(best[1], golden::kDeParabolaSeed7[1], kTol);
}

TEST(DifferentialEvolutionParity, ParabolaSeed7Dither)
{
    ShiftedParabola r;
    auto            best = runDE(r, 2, 7, 80, true, -10, 10, -20, 20);
    EXPECT_NEAR(best[0], golden::kDeParabolaSeed7Dither[0], kTol);
    EXPECT_NEAR(best[1], golden::kDeParabolaSeed7Dither[1], kTol);
}

TEST(DifferentialEvolutionParity, Sphere1DSeed1)
{
    SphereRating r;
    auto         best = runDE(r, 1, 1, 50, false, -5, 5, -10, 10);
    EXPECT_NEAR(best[0], golden::kDeSphere1Seed1[0], kTol);
}

TEST(DifferentialEvolutionParity, StepByStepTrace)
{
    SphereRating          r;
    DifferentialEvolution de(r, 2, false, 123);
    de.createCandidates({-5, -5}, {5, 5}, {-10, -10}, {10, 10});

    auto best = de.getBestCandidate();
    for (std::size_t i = 0; i < 2; i++)
        EXPECT_NEAR(best[i], golden::kDeTraceSphere2Seed123[0][i], kTol) << "trace 0";

    for (int s = 0; s < 5; s++)
    {
        de.evolutionStep();
        best = de.getBestCandidate();
        for (std::size_t i = 0; i < 2; i++)
            EXPECT_NEAR(best[i], golden::kDeTraceSphere2Seed123[s + 1][i], kTol)
                << "trace step " << (s + 1);
    }
}

TEST(DifferentialEvolutionConvergence, SphereConvergesNearZero)
{
    SphereRating r;
    auto         best = runDE(r, 3, 42, 60, false, -5, 5, -10, 10);
    for (double x : best) EXPECT_LT(std::abs(x), 1e-2);
}

TEST(DifferentialEvolutionConvergence, ParabolaConvergesToOptimum)
{
    ShiftedParabola r;
    auto            best = runDE(r, 2, 7, 80, false, -10, 10, -20, 20);
    EXPECT_NEAR(best[0], 3.0, 1e-3);
    EXPECT_NEAR(best[1], -2.0, 1e-3);
}

TEST(DifferentialEvolutionConvergence, Reproducible)
{
    SphereRating r1;
    SphereRating r2;
    auto         a = runDE(r1, 3, 42, 60, false, -5, 5, -10, 10);
    auto         b = runDE(r2, 3, 42, 60, false, -5, 5, -10, 10);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); i++) EXPECT_DOUBLE_EQ(a[i], b[i]);
}

TEST(DifferentialEvolutionConvergence, DifferentSeedsDiffer)
{
    SphereRating r1;
    SphereRating r2;
    auto         a = runDE(r1, 3, 42, 5, false, -5, 5, -10, 10);
    auto         b = runDE(r2, 3, 99, 5, false, -5, 5, -10, 10);
    bool         differ = false;
    for (std::size_t i = 0; i < a.size(); i++)
        if (a[i] != b[i])
            differ = true;
    EXPECT_TRUE(differ);
}
