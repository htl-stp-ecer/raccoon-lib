#include "localization/localization.hpp"

#include "odometry/angle_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace libstp::localization {

namespace {

inline float wrapAnglef(float angle_rad) {
    return static_cast<float>(libstp::odometry::wrapAngle(static_cast<double>(angle_rad)));
}

inline double wrapAngled(double angle_rad) {
    return libstp::odometry::wrapAngle(angle_rad);
}

inline double gaussianLogLikelihood(double error, double sigma) {
    if (!std::isfinite(sigma)) {
        return 0.0;
    }
    const double clampedSigma = std::max(sigma, 1e-6);
    const double normalized = error / clampedSigma;
    return -0.5 * normalized * normalized - std::log(clampedSigma);
}

inline double finiteSigmaOr(double sigma, double fallback) {
    if (!std::isfinite(sigma)) {
        return fallback;
    }
    return std::max(sigma, 1e-6);
}

}  // namespace

Localization::Localization(std::shared_ptr<libstp::odometry::IOdometry> odometry,
                           LocalizationConfig config,
                           std::optional<libstp::map::WorldMap> tableMap)
    : m_odometry(std::move(odometry)),
      m_config(config),
      m_rng(m_config.rng_seed),
      m_tableMap(std::move(tableMap)) {
    start();
}

Localization::~Localization() {
    stop();
}

libstp::foundation::Pose Localization::getPose() const {
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_worldPose;
}

void Localization::observe(const Observation& obs) {
    std::lock_guard<std::mutex> lk(m_mutex);

    if (!m_initialized) {
        initializeLocked(obs.pose);
    }
    integrateObservationLocked(obs);

    // Rebase the odom reference so the next tick measures delta from "now",
    // preventing the observation update from being immediately skewed by a
    // stale delta sampled before the resync point.
    if (m_odometry) {
        m_lastOdom = m_odometry->getPose();
        m_initialized = true;
    }
}

void Localization::start() {
    bool expected = false;
    if (!m_running.compare_exchange_strong(expected, true)) {
        return;  // already running
    }
    m_thread = std::thread([this] { tickLoop(); });
}

void Localization::stop() {
    bool expected = true;
    if (!m_running.compare_exchange_strong(expected, false)) {
        return;  // already stopped
    }
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void Localization::initializeLocked(const libstp::foundation::Pose& anchorPose) {
    initializeParticlesLocked(anchorPose, Eigen::Vector3d{1e-4, 1e-4, 1e-4});
    m_worldPose = anchorPose;
}

void Localization::initializeParticlesLocked(const libstp::foundation::Pose& anchorPose,
                                             const Eigen::Vector3d& sigma) {
    const int count = std::max(1, m_config.particle_count);
    m_particles.clear();
    m_particles.reserve(static_cast<size_t>(count));

    std::normal_distribution<double> noiseX(0.0, finiteSigmaOr(sigma.x(), 1e-4));
    std::normal_distribution<double> noiseY(0.0, finiteSigmaOr(sigma.y(), 1e-4));
    std::normal_distribution<double> noiseTheta(0.0, finiteSigmaOr(sigma.z(), 1e-4));
    const double uniformWeight = 1.0 / static_cast<double>(count);

    for (int i = 0; i < count; ++i) {
        Particle p;
        p.pose = anchorPose;
        p.pose.position.x() += static_cast<float>(noiseX(m_rng));
        p.pose.position.y() += static_cast<float>(noiseY(m_rng));
        p.pose.heading = wrapAnglef(anchorPose.heading + static_cast<float>(noiseTheta(m_rng)));
        p.weight = uniformWeight;
        m_particles.push_back(p);
    }
    m_worldPose = anchorPose;
}

void Localization::predictLocked(const libstp::foundation::Pose& odomDelta) {
    if (m_particles.empty()) {
        initializeLocked(m_worldPose);
    }

    const double translationalStep =
        std::hypot(static_cast<double>(odomDelta.position.x()),
                   static_cast<double>(odomDelta.position.y()));
    const double angularStep = std::abs(static_cast<double>(odomDelta.heading));
    const double sigmaXY =
        m_config.process_translation_noise_m +
        m_config.process_translation_noise_per_m * translationalStep;
    const double sigmaTheta =
        m_config.process_heading_noise_rad +
        m_config.process_heading_noise_per_rad * angularStep;

    std::normal_distribution<double> noiseX(0.0, std::max(sigmaXY, 1e-6));
    std::normal_distribution<double> noiseY(0.0, std::max(sigmaXY, 1e-6));
    std::normal_distribution<double> noiseTheta(0.0, std::max(sigmaTheta, 1e-6));

    for (auto& particle : m_particles) {
        particle.pose.position.x() +=
            odomDelta.position.x() + static_cast<float>(noiseX(m_rng));
        particle.pose.position.y() +=
            odomDelta.position.y() + static_cast<float>(noiseY(m_rng));
        particle.pose.heading = wrapAnglef(
            particle.pose.heading + odomDelta.heading + static_cast<float>(noiseTheta(m_rng)));
    }
    updateEstimateLocked();
}

void Localization::integrateObservationLocked(const Observation& obs) {
    if (m_particles.empty()) {
        initializeParticlesLocked(obs.pose, obs.sigma);
    }

    for (auto& particle : m_particles) {
        particle.weight = computeParticleLogWeight(particle, obs);
    }
    normalizeWeightsLocked();

    updateEstimateLocked();

    const double sumSquares = std::accumulate(
        m_particles.begin(), m_particles.end(), 0.0,
        [](double acc, const Particle& particle) {
            return acc + particle.weight * particle.weight;
        });
    const double ess = sumSquares > 0.0 ? 1.0 / sumSquares : 0.0;
    const double threshold =
        m_config.resample_effective_sample_ratio * static_cast<double>(m_particles.size());
    if (ess < threshold) {
        resampleLocked();
    }

    // Explicit resync observations must be able to relocate the cloud even when
    // the prior drifted too far away to leave enough support near the target.
    const int injectedCount = std::clamp(
        static_cast<int>(std::lround(
            m_config.observation_injection_ratio * static_cast<double>(m_particles.size()))),
        0,
        static_cast<int>(m_particles.size()));
    if (injectedCount > 0) {
        std::normal_distribution<double> obsX(0.0, finiteSigmaOr(obs.sigma.x(), 1e-4));
        std::normal_distribution<double> obsY(0.0, finiteSigmaOr(obs.sigma.y(), 1e-4));
        std::normal_distribution<double> obsTheta(0.0, finiteSigmaOr(obs.sigma.z(), 1e-4));
        for (int i = 0; i < injectedCount; ++i) {
            auto& particle = m_particles[static_cast<size_t>(i)];
            if (std::isfinite(obs.sigma.x())) {
                particle.pose.position.x() =
                    obs.pose.position.x() + static_cast<float>(obsX(m_rng));
            }
            if (std::isfinite(obs.sigma.y())) {
                particle.pose.position.y() =
                    obs.pose.position.y() + static_cast<float>(obsY(m_rng));
            }
            if (std::isfinite(obs.sigma.z())) {
                particle.pose.heading = wrapAnglef(
                    obs.pose.heading + static_cast<float>(obsTheta(m_rng)));
            }
        }
        const double uniformWeight = 1.0 / static_cast<double>(m_particles.size());
        for (auto& particle : m_particles) {
            particle.weight = uniformWeight;
        }

        for (auto& particle : m_particles) {
            particle.weight = computeParticleLogWeight(particle, obs);
        }
        normalizeWeightsLocked();
    }

    updateEstimateLocked();
}

double Localization::computeParticleLogWeight(const Particle& particle,
                                              const Observation& obs) const {
    double logWeight =
        gaussianLogLikelihood(
            static_cast<double>(particle.pose.position.x() - obs.pose.position.x()),
            obs.sigma.x()) +
        gaussianLogLikelihood(
            static_cast<double>(particle.pose.position.y() - obs.pose.position.y()),
            obs.sigma.y()) +
        gaussianLogLikelihood(
            wrapAngled(static_cast<double>(particle.pose.heading - obs.pose.heading)),
            obs.sigma.z());

    if (!m_tableMap || obs.surface_measurements.empty()) {
        return logWeight;
    }

    const float robotXCm = particle.pose.position.x() * 100.0f;
    const float robotYCm = particle.pose.position.y() * 100.0f;
    const float headingRad = particle.pose.heading;

    for (const auto& measurement : obs.surface_measurements) {
        const bool onSurface = measurement.kind == Observation::SurfaceKind::Line
                                   ? m_tableMap->sensorIsOnLine(
                                         robotXCm, robotYCm, headingRad, measurement.sensor)
                                   : m_tableMap->sensorIsOnWall(
                                         robotXCm, robotYCm, headingRad, measurement.sensor);
        const auto p =
            m_tableMap->sensorFieldPosition(robotXCm, robotYCm, headingRad, measurement.sensor);
        const double distanceCm = measurement.kind == Observation::SurfaceKind::Line
                                      ? static_cast<double>(
                                            m_tableMap->distanceToNearestLine(p.xCm, p.yCm))
                                      : static_cast<double>(
                                            m_tableMap->distanceToNearestWall(p.xCm, p.yCm));
        const double sigmaCm = std::max(measurement.sigma_cm, 1e-3);
        const double normalized = distanceCm / sigmaCm;
        const double proximityPenalty = 0.5 * normalized * normalized;

        if (measurement.detected) {
            logWeight += -proximityPenalty - std::log(sigmaCm);
            if (!onSurface) {
                logWeight -= 0.5;
            }
        } else if (onSurface) {
            logWeight -= 1.0 + proximityPenalty;
        } else {
            logWeight -= std::exp(-0.5 * normalized * normalized);
        }
    }

    return logWeight;
}

void Localization::normalizeWeightsLocked() {
    double maxLogWeight = -std::numeric_limits<double>::infinity();
    for (const auto& particle : m_particles) {
        maxLogWeight = std::max(maxLogWeight, particle.weight);
    }

    double weightSum = 0.0;
    for (auto& particle : m_particles) {
        particle.weight = std::exp(particle.weight - maxLogWeight);
        weightSum += particle.weight;
    }
    if (!(weightSum > 0.0) || !std::isfinite(weightSum)) {
        const double uniformWeight = 1.0 / static_cast<double>(m_particles.size());
        for (auto& particle : m_particles) {
            particle.weight = uniformWeight;
        }
        return;
    }
    for (auto& particle : m_particles) {
        particle.weight /= weightSum;
    }
}

void Localization::resampleLocked() {
    if (m_particles.empty()) {
        return;
    }

    std::vector<Particle> resampled;
    resampled.reserve(m_particles.size());

    const double step = 1.0 / static_cast<double>(m_particles.size());
    std::uniform_real_distribution<double> startDist(0.0, step);
    double r = startDist(m_rng);
    double c = m_particles.front().weight;
    size_t i = 0;

    for (size_t m = 0; m < m_particles.size(); ++m) {
        const double u = r + static_cast<double>(m) * step;
        while (u > c && i + 1 < m_particles.size()) {
            ++i;
            c += m_particles[i].weight;
        }
        Particle sample = m_particles[i];
        sample.weight = step;
        resampled.push_back(sample);
    }

    m_particles = std::move(resampled);
}

void Localization::updateEstimateLocked() {
    if (m_particles.empty()) {
        return;
    }

    double totalWeight = 0.0;
    double meanX = 0.0;
    double meanY = 0.0;
    double meanSin = 0.0;
    double meanCos = 0.0;

    for (const auto& particle : m_particles) {
        totalWeight += particle.weight;
        meanX += particle.weight * static_cast<double>(particle.pose.position.x());
        meanY += particle.weight * static_cast<double>(particle.pose.position.y());
        meanSin += particle.weight * std::sin(static_cast<double>(particle.pose.heading));
        meanCos += particle.weight * std::cos(static_cast<double>(particle.pose.heading));
    }

    if (!(totalWeight > 0.0)) {
        return;
    }

    m_worldPose.position.x() = static_cast<float>(meanX / totalWeight);
    m_worldPose.position.y() = static_cast<float>(meanY / totalWeight);
    m_worldPose.heading = wrapAnglef(static_cast<float>(std::atan2(meanSin, meanCos)));
}

void Localization::tickLoop() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    const auto period = std::chrono::milliseconds(m_config.tick_period_ms);

    while (m_running.load(std::memory_order_acquire)) {
        if (m_odometry) {
            const auto cur = m_odometry->getPose();
            std::lock_guard<std::mutex> lk(m_mutex);
            if (!m_initialized) {
                m_lastOdom = cur;
                initializeLocked(cur);
                m_initialized = true;
            } else {
                // Reset / impossible-jump detection.
                //
                // Until Phase 4 lifts ``odometry.reset()`` out of motion
                // ``start()``, the localization pass-through has to survive
                // the discrete jump that a reset injects into the underlying
                // odometry. IOdometry does not (yet) expose a monotonic
                // reset counter, so we detect the reset heuristically:
                //
                //   * reset signature: the new sample lands exactly at the
                //     origin while the previous sample was meaningfully off
                //     it. That is the only way ``Stm32Odometry::reset()``
                //     leaves the pose.
                //   * defensive: any single-tick jump greater than
                //     ``kMaxPlausibleStep`` (0.5 m at 100 Hz ⇒ 50 m/s) is
                //     physically impossible for the platform and is treated
                //     the same way — rebase, do not accumulate.
                //
                // Both branches collapse to "swallow the delta and rebase
                // ``m_lastOdom``" so the next genuine motion delta is
                // measured from the post-reset reference. The world pose
                // stays put, which is exactly the Phase-2 promise.
                constexpr float kOriginEps = 1e-4f;
                constexpr float kMeaningfulEps = 1e-3f;
                constexpr float kMaxPlausibleStep = 0.5f;

                const bool currentIsOrigin =
                    cur.position.norm() < kOriginEps &&
                    std::abs(cur.heading) < kOriginEps;
                const bool lastWasMeaningful =
                    m_lastOdom.position.norm() > kMeaningfulEps ||
                    std::abs(m_lastOdom.heading) > kMeaningfulEps;
                const float stepNorm =
                    (cur.position - m_lastOdom.position).norm();
                const bool jumpTooBig = stepNorm > kMaxPlausibleStep;

                if ((currentIsOrigin && lastWasMeaningful) || jumpTooBig) {
                    // Rebase silently. World pose is preserved; the next
                    // tick will measure delta from the new ``cur``.
                    m_lastOdom = cur;
                } else {
                    libstp::foundation::Pose odomDelta;
                    odomDelta.position.x() = cur.position.x() - m_lastOdom.position.x();
                    odomDelta.position.y() = cur.position.y() - m_lastOdom.position.y();
                    odomDelta.heading = wrapAnglef(cur.heading - m_lastOdom.heading);
                    predictLocked(odomDelta);

                    m_lastOdom = cur;
                }
            }
        }

        next += period;
        // Guard against drift: if we fell badly behind (stalled), reset the schedule.
        const auto now = clock::now();
        if (next < now) {
            next = now;
        }
        std::this_thread::sleep_until(next);
    }
}

}  // namespace libstp::localization
