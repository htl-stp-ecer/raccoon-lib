#pragma once

// Particle resampling algorithms.
//
// Ported (and adapted to std::mt19937) from MRPT's CParticleFilterCapable
// (BSD-3-Clause, https://github.com/MRPT/mrpt). Four schemes are provided —
// Multinomial, Residual, Stratified, Systematic. References:
//   - Doucet & Johansen, "A tutorial on particle filtering and smoothing" (2009)
//   - Liu & Chen, "Sequential Monte Carlo methods for dynamic systems" (1998)
//   - Kitagawa, "Monte Carlo filter ..." (1996)
//   - Carpenter, Clifford, Fearnhead, "Improved particle filter ..." (1999)
//
// All variants take log-weights (need not be normalized) and emit indexes the
// caller plugs into its own particle buffer via performSubstitution-style copy.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace libstp::localization {

enum class ResamplingAlgorithm : uint8_t {
    Multinomial = 0,  ///< Classical: M independent samples from the CDF.
    Residual = 1,     ///< Deterministic floor(M*w) part + multinomial remainder.
    Stratified = 2,   ///< Strata of size 1/M, one uniform draw per stratum.
    Systematic = 3,   ///< Single uniform offset + fixed-step traversal (lowest variance).
};

/**
 * Compute resampling indexes from log-weights.
 *
 * @param method     One of @ref ResamplingAlgorithm.
 * @param logWeights Per-particle log-weights (unnormalized).
 * @param rng        Random source.
 * @param outIndexes Filled with indexes drawn from logWeights. Resized to
 *                   outCount (or logWeights.size() if outCount == 0).
 * @param outCount   Desired output count; 0 means "same as input".
 */
inline void computeResampling(ResamplingAlgorithm method,
                              const std::vector<double>& logWeights,
                              std::mt19937& rng,
                              std::vector<size_t>& outIndexes,
                              size_t outCount = 0) {
    const size_t M = logWeights.size();
    if (M == 0) {
        outIndexes.clear();
        return;
    }
    if (outCount == 0) {
        outCount = M;
    }

    // Log-sum-exp normalization to a linear weight vector.
    double maxLog = logWeights.front();
    for (const double lw : logWeights) {
        maxLog = std::max(maxLog, lw);
    }
    std::vector<double> linW(M);
    double sum = 0.0;
    for (size_t i = 0; i < M; ++i) {
        linW[i] = std::exp(logWeights[i] - maxLog);
        sum += linW[i];
    }
    if (!(sum > 0.0)) {
        // Degenerate input: uniform fallback so callers never see UB.
        outIndexes.assign(outCount, 0);
        std::uniform_int_distribution<size_t> uniDist(0, M - 1);
        for (auto& idx : outIndexes) {
            idx = uniDist(rng);
        }
        return;
    }
    const double inv = 1.0 / sum;
    for (auto& w : linW) {
        w *= inv;
    }

    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    auto cumulativeWalk = [&](const std::vector<double>& T) {
        // Shared CDF traversal used by Multinomial / Stratified / Systematic.
        std::vector<double> Q(M);
        double acc = 0.0;
        for (size_t i = 0; i < M; ++i) {
            acc += linW[i];
            Q[i] = acc;
        }
        Q[M - 1] = 1.1;  // Guard against FP-rounding past 1.0.

        outIndexes.resize(outCount);
        size_t i = 0;
        size_t j = 0;
        while (i < outCount) {
            if (T[i] < Q[j]) {
                outIndexes[i++] = j;
            } else if (j + 1 < M) {
                ++j;
            } else {
                outIndexes[i++] = j;  // Saturate at last bin.
            }
        }
    };

    switch (method) {
        case ResamplingAlgorithm::Multinomial: {
            std::vector<double> T(outCount);
            for (auto& t : T) {
                t = uni01(rng);
            }
            std::sort(T.begin(), T.end());
            cumulativeWalk(T);
            break;
        }

        case ResamplingAlgorithm::Residual: {
            std::vector<uint32_t> N(M, 0);
            size_t R = 0;
            for (size_t i = 0; i < M; ++i) {
                N[i] = static_cast<uint32_t>(static_cast<double>(outCount) * linW[i]);
                R += N[i];
            }
            outIndexes.resize(outCount);
            size_t j = 0;
            for (size_t i = 0; i < M && j < outCount; ++i) {
                for (uint32_t k = 0; k < N[i] && j < outCount; ++k) {
                    outIndexes[j++] = i;
                }
            }
            const size_t residualCount = outCount - j;
            if (residualCount == 0) {
                break;
            }
            // Residual part: modified weights = M * w_i - N_i, multinomial.
            std::vector<double> linWmod(M);
            const double invRes = 1.0 / static_cast<double>(residualCount);
            for (size_t i = 0; i < M; ++i) {
                linWmod[i] =
                    invRes * (static_cast<double>(outCount) * linW[i] - static_cast<double>(N[i]));
            }
            std::vector<double> Qres(M);
            double acc = 0.0;
            for (size_t i = 0; i < M; ++i) {
                acc += linWmod[i];
                Qres[i] = acc;
            }
            Qres[M - 1] = 1.1;
            std::vector<double> T(residualCount);
            for (auto& t : T) {
                t = uni01(rng);
            }
            std::sort(T.begin(), T.end());
            size_t walk = 0;
            size_t k = 0;
            while (walk < residualCount) {
                if (T[walk] < Qres[k]) {
                    outIndexes[j + walk] = k;
                    ++walk;
                } else if (k + 1 < M) {
                    ++k;
                } else {
                    outIndexes[j + walk] = k;
                    ++walk;
                }
            }
            break;
        }

        case ResamplingAlgorithm::Stratified: {
            std::vector<double> T(outCount);
            const double step = 1.0 / static_cast<double>(outCount);
            for (size_t i = 0; i < outCount; ++i) {
                T[i] = (static_cast<double>(i) + uni01(rng)) * step;
            }
            cumulativeWalk(T);
            break;
        }

        case ResamplingAlgorithm::Systematic: {
            std::vector<double> T(outCount);
            const double step = 1.0 / static_cast<double>(outCount);
            const double offset = uni01(rng) * step;
            for (size_t i = 0; i < outCount; ++i) {
                T[i] = offset + static_cast<double>(i) * step;
            }
            cumulativeWalk(T);
            break;
        }
    }
}

/**
 * Effective sample size (ESS) computed from log-weights, normalized to [0, 1].
 *
 * Log-sum-exp trick to dodge underflow on tightly peaked weight distributions.
 * Ported from MRPT's CParticleFilterDataImpl::ESS.
 */
inline double effectiveSampleRatio(const std::vector<double>& logWeights) {
    const size_t M = logWeights.size();
    if (M == 0) {
        return 0.0;
    }
    double maxLog = logWeights.front();
    for (const double lw : logWeights) {
        maxLog = std::max(maxLog, lw);
    }
    double sumLin = 0.0;
    for (const double lw : logWeights) {
        sumLin += std::exp(lw - maxLog);
    }
    if (!(sumLin > 0.0)) {
        return 0.0;
    }
    double sumSq = 0.0;
    for (const double lw : logWeights) {
        const double w = std::exp(lw - maxLog) / sumLin;
        sumSq += w * w;
    }
    if (!(sumSq > 0.0)) {
        return 0.0;
    }
    return 1.0 / (static_cast<double>(M) * sumSq);
}

}  // namespace libstp::localization
