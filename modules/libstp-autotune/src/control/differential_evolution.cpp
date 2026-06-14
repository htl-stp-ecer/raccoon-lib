#include "autotune/control/differential_evolution.hpp"

#include <cmath>

namespace libstp::autotune::control
{

// mulberry32 — must reproduce the JS reference bit-for-bit. JS `Math.imul`
// is signed 32-bit multiply with wraparound; the C++ equivalent is a
// uint32_t multiply (modulo 2^32). All shifts here are logical (>>>) so we
// operate on uint32_t throughout, then divide by 2^32 to land in [0, 1).
double Rng::next()
{
    state_ += 0x6D2B79F5u; // (a + 0x6D2B79F5) | 0
    std::uint32_t a = state_;

    std::uint32_t t = a ^ (a >> 15);
    t *= (1u | a); // Math.imul(a ^ (a >>> 15), 1 | a)

    std::uint32_t t2 = t ^ (t >> 7);
    t2 *= (61u | t);   // Math.imul(t ^ (t >>> 7), 61 | t)
    t = (t2 + t) ^ t;  // (... + t) ^ t   [JS: t = (t + Math.imul(...)) ^ t]

    const std::uint32_t r = (t ^ (t >> 14));
    return static_cast<double>(r) / 4294967296.0;
}

// ---------------------------------------------------------------------------

DifferentialEvolution::DifferentialEvolution(IDifferentialEvolutionRating& rating,
                                             int                           numParams,
                                             bool                          dither,
                                             std::uint32_t                 seed)
    : rating_(rating), num_params_(numParams), dither_(dither), rng_(seed)
{
    const std::size_t numCandidates =
        static_cast<std::size_t>(numParams) * static_cast<std::size_t>(kNumCandidatesFactor);

    candidate_list_.assign(numCandidates, Candidate(numParams));
    temp_candidate_list_.assign(numCandidates, Candidate(numParams));
}

void DifferentialEvolution::createCandidate(Candidate&                 c,
                                            const std::vector<double>& minParams,
                                            const std::vector<double>& maxParams)
{
    for (std::size_t idx = 0; idx < c.paramList.size(); idx++)
    {
        c.paramList[idx] = minParams[idx] + rng_.next() * (maxParams[idx] - minParams[idx]);
    }
}

DifferentialEvolution::Candidate DifferentialEvolution::mutateAndCrossover(
    const Candidate&           self,
    const Candidate&           a,
    const Candidate&           b,
    const Candidate&           c,
    double                     weightingFactor,
    double                     crossoverProbability,
    const std::vector<double>& minParams,
    const std::vector<double>& maxParams)
{
    Candidate         newCandidate(num_params_);
    const std::size_t randomIdx =
        static_cast<std::size_t>(std::floor(rng_.next() * static_cast<double>(self.paramList.size())));

    for (std::size_t idx = 0; idx < self.paramList.size(); idx++)
    {
        const double r = rng_.next();

        if ((idx == randomIdx) || (r < crossoverProbability))
        {
            double newParam =
                c.paramList[idx] + weightingFactor * (b.paramList[idx] - a.paramList[idx]);

            if (newParam < minParams[idx])
                newParam = minParams[idx];
            if (newParam > maxParams[idx])
                newParam = maxParams[idx];

            newCandidate.paramList[idx] = newParam;
        }
        else
        {
            newCandidate.paramList[idx] = self.paramList[idx];
        }
    }

    return newCandidate;
}

std::vector<double> DifferentialEvolution::getBestCandidate() const
{
    const Candidate* best = &candidate_list_[0];

    for (std::size_t idx = 1; idx < candidate_list_.size(); idx++)
    {
        if (candidate_list_[idx].rating < best->rating)
            best = &candidate_list_[idx];
    }

    return best->paramList;
}

void DifferentialEvolution::createCandidates(const std::vector<double>& initMinParams,
                                             const std::vector<double>& initMaxParams,
                                             const std::vector<double>& limitMinParams,
                                             const std::vector<double>& limitMaxParams)
{
    limit_min_params_ = limitMinParams;
    limit_max_params_ = limitMaxParams;

    for (auto& c : candidate_list_)
    {
        createCandidate(c, initMinParams, initMaxParams);
        c.rating = rating_.rateCandidate(c.paramList);
    }
}

void DifferentialEvolution::evolutionStep()
{
    double weightingFactor;

    if (dither_)
        weightingFactor =
            kWeightingFactorMin + rng_.next() * (kWeightingFactorMax - kWeightingFactorMin);
    else
        weightingFactor = kWeightingFactorFix;

    for (std::size_t idx = 0; idx < candidate_list_.size(); idx++)
    {
        std::size_t idxA, idxB, idxC;
        getThreeRandomCandidates(idx, idxA, idxB, idxC);

        Candidate newCandidate =
            mutateAndCrossover(candidate_list_[idx], candidate_list_[idxA], candidate_list_[idxB],
                               candidate_list_[idxC], weightingFactor, kCrossoverProbability,
                               limit_min_params_, limit_max_params_);

        newCandidate.rating = rating_.rateCandidate(newCandidate.paramList);

        if (newCandidate.rating < candidate_list_[idx].rating)
            temp_candidate_list_[idx] = newCandidate;
        else
            temp_candidate_list_[idx] = candidate_list_[idx];
    }

    for (std::size_t idx = 0; idx < candidate_list_.size(); idx++)
        candidate_list_[idx] = temp_candidate_list_[idx];
}

std::size_t DifferentialEvolution::getRandomCandidateIndex()
{
    return static_cast<std::size_t>(
        std::floor(rng_.next() * static_cast<double>(candidate_list_.size())));
}

void DifferentialEvolution::getThreeRandomCandidates(std::size_t  currentIdx,
                                                     std::size_t& idxA,
                                                     std::size_t& idxB,
                                                     std::size_t& idxC)
{
    do
    {
        idxA = getRandomCandidateIndex();
    } while (idxA == currentIdx);

    do
    {
        idxB = getRandomCandidateIndex();
    } while ((idxB == currentIdx) || (idxB == idxA));

    do
    {
        idxC = getRandomCandidateIndex();
    } while ((idxC == currentIdx) || (idxC == idxA) || (idxC == idxB));
}

} // namespace libstp::autotune::control
