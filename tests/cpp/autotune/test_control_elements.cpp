// Numerical-parity tests for the ported control elements. Golden values are
// generated from the original TypeScript sources (see control_golden.hpp /
// /tmp/ct-harness/gen.ts) and compared to ~1e-9.
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "autotune/control/control_elements.hpp"
#include "autotune/control/control_loop.hpp"
#include "autotune/control/control_path.hpp"
#include "control_golden.hpp"

using namespace libstp::autotune::control;

namespace
{
constexpr double kTol = 1e-9;
constexpr double kDt  = 0.1;

std::vector<double> stepInput(std::size_t n)
{
    return std::vector<double>(n, 1.0);
}

const std::vector<double> kVarSeq = {0.0, 0.5, 1.0,  1.0, 0.8, 0.3,
                                     -0.2, -0.2, 0.0, 0.6, 0.6, 0.6};

std::vector<double> run(ControlElement& el, const std::vector<double>& inputs, double dt)
{
    el.initialize(dt);
    std::vector<double> ys;
    ys.reserve(inputs.size());
    for (double u : inputs) ys.push_back(el.calculate(u, dt));
    return ys;
}

void expectClose(const std::vector<double>& got, const std::vector<double>& want)
{
    ASSERT_EQ(got.size(), want.size());
    for (std::size_t i = 0; i < got.size(); i++)
        EXPECT_NEAR(got[i], want[i], kTol) << "index " << i;
}
} // namespace

TEST(ControlElementParity, PStep)
{
    ControlElementP el(2.5);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_P);
}

TEST(ControlElementParity, IStep)
{
    ControlElementI el(0.5);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_I);
}

TEST(ControlElementParity, DStep)
{
    ControlElementD el(1.5);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_D);
}

TEST(ControlElementParity, PT1Step)
{
    ControlElementPT1 el(2.0, 0.7);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_PT1);
}

TEST(ControlElementParity, PT2Step)
{
    ControlElementPT2 el(2.0, 0.7, 0.5);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_PT2);
}

TEST(ControlElementParity, PD1Step)
{
    ControlElementPD1 el(2.0, 0.3);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_PD1);
}

TEST(ControlElementParity, DeadTimeStep)
{
    ControlElementDeadTime el(0.45);
    expectClose(run(el, stepInput(20), kDt), golden::kElemStep_DeadTime);
}

TEST(ControlElementParity, VarSequence)
{
    {
        ControlElementD el(1.5);
        expectClose(run(el, kVarSeq, kDt), golden::kElemVar_D);
    }
    {
        ControlElementPD1 el(2.0, 0.3);
        expectClose(run(el, kVarSeq, kDt), golden::kElemVar_PD1);
    }
    {
        ControlElementDeadTime el(0.3);
        expectClose(run(el, kVarSeq, kDt), golden::kElemVar_DeadTime);
    }
    {
        ControlElementI el(0.5);
        expectClose(run(el, kVarSeq, kDt), golden::kElemVar_I);
    }
    {
        ControlElementPT1 el(2.0, 0.7);
        expectClose(run(el, kVarSeq, kDt), golden::kElemVar_PT1);
    }
    {
        ControlElementPT2 el(2.0, 0.7, 0.5);
        expectClose(run(el, kVarSeq, kDt), golden::kElemVar_PT2);
    }
}

TEST(ControlElementParity, DeadTimeZeroAndLargeDelay)
{
    {
        ControlElementDeadTime el(0.0);
        expectClose(run(el, kVarSeq, kDt), golden::kDeadTimeZero);
    }
    {
        // Delay (2.0 s) larger than the input window (1.2 s): all-zero output.
        ControlElementDeadTime el(2.0);
        expectClose(run(el, kVarSeq, kDt), golden::kDeadTimeLarge);
    }
}

TEST(ControlPathParity, TwoStagePT1Step)
{
    ControlPath path;
    path.add(std::make_unique<ControlElementPT1>(1.0, 0.5));
    path.add(std::make_unique<ControlElementPT1>(1.0, 0.3));
    expectClose(run(path, stepInput(20), kDt), golden::kPath2);
}

TEST(ControlElementPIDTest, DtExplicitFirmwareMatch)
{
    // kp=2, ki=0.5, kd=1, dt=0.5, e=[1,1,1,0,-1].
    // iErr += e*dt; dErr = (e-prev)/dt; u = kp*e + ki*iErr + kd*dErr.
    ControlElementPID pid(2.0, 0.5, 1.0);
    const std::vector<double> errs = {1.0, 1.0, 1.0, 0.0, -1.0};
    const std::vector<double> want = {4.25, 2.5, 2.75, -1.25, -3.5};

    pid.initialize(0.5);
    std::vector<double> got;
    got.reserve(errs.size());
    for (double e : errs) got.push_back(pid.calculate(e, 0.5));
    expectClose(got, want);

    // initialize() resets state — re-running reproduces the same outputs.
    pid.initialize(0.5);
    std::vector<double> got2;
    got2.reserve(errs.size());
    for (double e : errs) got2.push_back(pid.calculate(e, 0.5));
    expectClose(got2, want);

    // optimizeSetParameters loads [kp, ki, kd] in order.
    ControlElementPID pid2(0.0, 0.0, 0.0);
    pid2.optimizeSetParameters({2.0, 0.5, 1.0}, 0);
    pid2.initialize(0.5);
    std::vector<double> got3;
    got3.reserve(errs.size());
    for (double e : errs) got3.push_back(pid2.calculate(e, 0.5));
    expectClose(got3, want);
}

TEST(ControlLoopParity, PControllerPT1Plant)
{
    ControlElementP   controller(2.0);
    ControlElementPT1 plant(1.0, 0.4);
    ControlLoop       loop(controller, plant);

    loop.initialize(kDt);
    std::vector<double> ys;
    std::vector<double> us;
    double              w = 0;
    for (int i = 0; i < 40; i++)
    {
        if (i >= 2)
            w = 1.0;
        ys.push_back(loop.calculate(w, kDt));
        us.push_back(loop.u());
    }

    expectClose(ys, golden::kLoopY);
    expectClose(us, golden::kLoopU);
    EXPECT_NEAR(loop.yMin(), golden::kLoopYMin, kTol);
    EXPECT_NEAR(loop.yMax(), golden::kLoopYMax, kTol);
    EXPECT_NEAR(loop.amplitude(), golden::kLoopAmplitude, kTol);
}
