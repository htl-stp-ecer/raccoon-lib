#pragma once

#include <cstdint>
#include <vector>

// Port of Framework/DifferentialEvolution/DifferentialEvolution.ts and
// IDifferentialEvolutionRating.ts — a DE/rand/1/bin global optimizer.
//
// DETERMINISM: the TypeScript original calls the non-seedable Math.random().
// To make parity tests reproducible, randomness is injected through a
// constructor-provided seed driving a mulberry32 PRNG (Rng below). mulberry32
// is bit-exact across the JS golden-value harness and this C++ implementation,
// so a given (seed, rating) pair yields identical optimizer trajectories in
// both. The DE algorithm itself is unchanged from the TS:
//   NUM_CANDIDATES_FACTOR = 12, WEIGHTING_FACTOR_FIX = 0.8,
//   dither in [0.5, 1.0], CROSSOVER_PROBABILITY = 0.8, greedy selection,
//   distinct A/B/C index draws, guaranteed mutation at a random index.

namespace libstp::autotune::control
{

/// Cost function to minimise.  Port of IDifferentialEvolutionRating.
class IDifferentialEvolutionRating
{
public:
    virtual ~IDifferentialEvolutionRating()                              = default;
    virtual double rateCandidate(const std::vector<double>& paramList)   = 0;
};

/**
 * @brief mulberry32 PRNG matching the JS golden-value harness exactly.
 *
 * Returns doubles in [0, 1). Identical integer arithmetic (32-bit wrapping,
 * Math.imul semantics) guarantees bit-for-bit parity with the reference run.
 */
class Rng
{
public:
    explicit Rng(std::uint32_t seed) : state_(seed) {}

    /// Uniform double in [0, 1), bit-identical to the JS mulberry32.
    double next();

private:
    std::uint32_t state_;
};

/**
 * @brief DE/rand/1/bin optimizer.  Port of DifferentialEvolution.ts.
 */
class DifferentialEvolution
{
public:
    /**
     * @param rating    cost function (minimised).
     * @param numParams problem dimensionality.
     * @param dither     enable per-step random weighting factor.
     * @param seed       PRNG seed (defaults to a fixed value for reproducibility).
     */
    DifferentialEvolution(IDifferentialEvolutionRating& rating,
                          int                           numParams,
                          bool                          dither = false,
                          std::uint32_t                 seed   = 0xC0FFEEu);

    /// Best (lowest-rated) parameter vector found so far.
    [[nodiscard]] std::vector<double> getBestCandidate() const;

    /// Seed the population uniformly in [initMin, initMax] and rate it.
    void createCandidates(const std::vector<double>& initMinParams,
                          const std::vector<double>& initMaxParams,
                          const std::vector<double>& limitMinParams,
                          const std::vector<double>& limitMaxParams);

    /// One generation of mutation / crossover / greedy selection.
    void evolutionStep();

private:
    struct Candidate
    {
        std::vector<double> paramList;
        double              rating{0};

        explicit Candidate(int numParams)
            : paramList(static_cast<std::size_t>(numParams), 0.0)
        {
        }
    };

    static constexpr int    kNumCandidatesFactor = 12;
    static constexpr double kWeightingFactorMin  = 0.5;
    static constexpr double kWeightingFactorMax  = 1.0;
    static constexpr double kWeightingFactorFix  = 0.8;
    static constexpr double kCrossoverProbability = 0.8;

    void createCandidate(Candidate&                 c,
                         const std::vector<double>& minParams,
                         const std::vector<double>& maxParams);

    [[nodiscard]] Candidate mutateAndCrossover(const Candidate& self,
                                               const Candidate& a,
                                               const Candidate& b,
                                               const Candidate& c,
                                               double           weightingFactor,
                                               double           crossoverProbability,
                                               const std::vector<double>& minParams,
                                               const std::vector<double>& maxParams);

    [[nodiscard]] std::size_t getRandomCandidateIndex();
    void getThreeRandomCandidates(std::size_t  currentIdx,
                                  std::size_t& idxA,
                                  std::size_t& idxB,
                                  std::size_t& idxC);

    IDifferentialEvolutionRating& rating_;
    int                           num_params_;
    bool                          dither_;
    Rng                           rng_;

    std::vector<Candidate> candidate_list_;
    std::vector<Candidate> temp_candidate_list_;

    std::vector<double> limit_min_params_;
    std::vector<double> limit_max_params_;
};

} // namespace libstp::autotune::control
