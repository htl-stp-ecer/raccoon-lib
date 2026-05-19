// Tests for the resampling helpers in localization/resampling.hpp.
//
// Adapted from MRPT's bayes_unittest.cpp (BSD-3-Clause). The MRPT tests target
// CParticleFilterCapable's class-based API; here we exercise the free-function
// port that powers libstp::localization::Localization::resampleLocked() and
// the diagnostics' ESS computation.

#include <gtest/gtest.h>

#include "localization/resampling.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

using libstp::localization::computeResampling;
using libstp::localization::effectiveSampleRatio;
using libstp::localization::ResamplingAlgorithm;

namespace {

// Deterministic RNG so every test is repeatable.
std::mt19937 makeRng(uint32_t seed = 0xC0FFEEu) {
    return std::mt19937{seed};
}

void expectUniformCoverage(ResamplingAlgorithm method) {
    constexpr size_t N = 50;
    std::vector<double> logW(N, 0.0);  // uniform weights
    std::vector<size_t> idx;
    auto rng = makeRng();
    computeResampling(method, logW, rng, idx);

    ASSERT_EQ(idx.size(), N);
    for (const size_t i : idx) {
        EXPECT_LT(i, N);
    }
}

}  // namespace

TEST(Resampling, MultinomialUniformCoversAllParticles) {
    expectUniformCoverage(ResamplingAlgorithm::Multinomial);
}

TEST(Resampling, ResidualUniformCoversAllParticles) {
    expectUniformCoverage(ResamplingAlgorithm::Residual);
}

TEST(Resampling, StratifiedUniformCoversAllParticles) {
    expectUniformCoverage(ResamplingAlgorithm::Stratified);
}

TEST(Resampling, SystematicUniformCoversAllParticles) {
    expectUniformCoverage(ResamplingAlgorithm::Systematic);
}

// One particle holds essentially all the mass — every method must pick it.
TEST(Resampling, DominatedWeightCollapsesAllMethods) {
    constexpr size_t N = 20;
    std::vector<double> logW(N, -100.0);
    logW[2] = 0.0;

    for (const auto method : {
             ResamplingAlgorithm::Multinomial, ResamplingAlgorithm::Residual,
             ResamplingAlgorithm::Stratified, ResamplingAlgorithm::Systematic}) {
        std::vector<size_t> idx;
        auto rng = makeRng();
        computeResampling(method, logW, rng, idx);
        ASSERT_EQ(idx.size(), N) << "method=" << static_cast<int>(method);
        for (const size_t i : idx) {
            EXPECT_EQ(i, 2u) << "method=" << static_cast<int>(method);
        }
    }
}

// outCount=0 means "match input size".
TEST(Resampling, OutCountDefaultsToInputSize) {
    std::vector<double> logW(7, 0.0);
    std::vector<size_t> idx;
    auto rng = makeRng();
    computeResampling(ResamplingAlgorithm::Systematic, logW, rng, idx);
    EXPECT_EQ(idx.size(), logW.size());
}

// outCount can shrink (downsample) or grow (upsample).
TEST(Resampling, OutCountCanDifferFromInput) {
    std::vector<double> logW(10, 0.0);
    std::vector<size_t> idx;
    auto rng = makeRng();
    computeResampling(ResamplingAlgorithm::Systematic, logW, rng, idx, /*outCount=*/25);
    EXPECT_EQ(idx.size(), 25u);
    for (const size_t i : idx) {
        EXPECT_LT(i, logW.size());
    }
}

// Statistical property of low-variance methods: with weights ~ truth, the
// empirical histogram should track the weights closely.
TEST(Resampling, SystematicHistogramTracksWeights) {
    // Three-peaked distribution: 70% / 20% / 10%.
    std::vector<double> linW = {0.7, 0.2, 0.1, 0.0, 0.0};
    std::vector<double> logW(linW.size());
    std::transform(linW.begin(), linW.end(), logW.begin(),
                   [](double w) { return w > 0.0 ? std::log(w) : -std::numeric_limits<double>::infinity(); });

    constexpr size_t N = 1000;
    std::vector<size_t> idx;
    auto rng = makeRng(42);
    computeResampling(ResamplingAlgorithm::Systematic, logW, rng, idx, N);
    ASSERT_EQ(idx.size(), N);

    std::vector<size_t> hist(linW.size(), 0);
    for (const size_t i : idx) {
        ++hist[i];
    }
    EXPECT_NEAR(static_cast<double>(hist[0]) / N, 0.7, 0.02);
    EXPECT_NEAR(static_cast<double>(hist[1]) / N, 0.2, 0.02);
    EXPECT_NEAR(static_cast<double>(hist[2]) / N, 0.1, 0.02);
    EXPECT_EQ(hist[3], 0u);
    EXPECT_EQ(hist[4], 0u);
}

// All-zero input would otherwise blow up — the helper must fall back gracefully.
TEST(Resampling, AllNegInfWeightsFallsBackToUniform) {
    std::vector<double> logW(8, -std::numeric_limits<double>::infinity());
    std::vector<size_t> idx;
    auto rng = makeRng();
    computeResampling(ResamplingAlgorithm::Systematic, logW, rng, idx);
    EXPECT_EQ(idx.size(), logW.size());
    for (const size_t i : idx) {
        EXPECT_LT(i, logW.size());
    }
}

TEST(Resampling, EmptyInputProducesEmptyOutput) {
    std::vector<double> logW;
    std::vector<size_t> idx{1, 2, 3};
    auto rng = makeRng();
    computeResampling(ResamplingAlgorithm::Systematic, logW, rng, idx);
    EXPECT_TRUE(idx.empty());
}

// ESS — direct port of MRPT bayes_unittest.cpp ESS tests.

TEST(ResamplingESS, UniformWeightsGiveOne) {
    std::vector<double> logW(100, 0.0);
    EXPECT_NEAR(effectiveSampleRatio(logW), 1.0, 1e-10);
}

TEST(ResamplingESS, SingleDominantWeightApproachesZero) {
    std::vector<double> logW(100, -100.0);
    logW[0] = 0.0;
    EXPECT_LT(effectiveSampleRatio(logW), 0.1);
}

TEST(ResamplingESS, EmptyInputReturnsZero) {
    std::vector<double> logW;
    EXPECT_DOUBLE_EQ(effectiveSampleRatio(logW), 0.0);
}

TEST(ResamplingESS, AllNegInfReturnsZero) {
    std::vector<double> logW(10, -std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(effectiveSampleRatio(logW), 0.0);
}

TEST(ResamplingESS, TwoEqualWeightsGiveHalf) {
    // ESS for [1, 1, 0, 0, ...] should be 2/N.
    std::vector<double> logW(10, -std::numeric_limits<double>::infinity());
    logW[0] = 0.0;
    logW[1] = 0.0;
    EXPECT_NEAR(effectiveSampleRatio(logW), 2.0 / 10.0, 1e-12);
}
