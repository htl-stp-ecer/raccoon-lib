#pragma once

#include "foundation/types.hpp"
#include "libstp/map/Geometry.hpp"
#include "libstp/map/WorldMap.hpp"
#include "hal/odometry.hpp"
#include "threading/thread_manager.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stop_token>
#include <string>
#include <vector>

namespace libstp::localization {

/**
 * @brief Sigma-weighted pose observation pushed by Resync-Steps.
 *
 * `sigma` is the measurement standard deviation per axis (x, y, theta).
 * Infinite sigma leaves that axis untouched.
 */
struct Observation {
    enum class SurfaceKind : uint8_t {
        Line = 0,
        Wall = 1,
    };

    struct SurfaceMeasurement {
        SurfaceKind kind{SurfaceKind::Line};
        libstp::map::SensorOffset sensor{};
        bool detected{true};
        double sigma_cm{1.0};
    };

    libstp::foundation::Pose pose{};
    Eigen::Vector3d sigma{1e-3, 1e-3, 1e-3};
    std::vector<SurfaceMeasurement> surface_measurements;
};

struct LocalizationConfig {
    int tick_period_ms{10};  ///< 100 Hz default
    int particle_count{128};
    double process_translation_noise_m{0.002};
    double process_translation_noise_per_m{0.02};
    double process_heading_noise_rad{0.01};
    double process_heading_noise_per_rad{0.05};
    double observation_injection_ratio{0.35};
    double resample_effective_sample_ratio{0.5};
    uint32_t rng_seed{0x5EED1234u};
};

/**
 * @brief Thread-safe particle-filter localization service.
 *
 * Owns a background thread that polls the supplied IOdometry at a fixed rate,
 * propagates a particle cloud from the per-tick odometry delta, and exposes the
 * estimated world pose via @ref getPose(). @ref observe() lets resync sites
 * inject sigma-weighted absolute pose observations.
 *
 * This Phase-6 increment keeps the existing IO contract but replaces the
 * pass-through state with a low-variance-resampled particle filter. Explicit
 * pose observations become functional; sensor/map likelihoods can plug into
 * the same particle representation later.
 *
 * Lifecycle:
 *   - The constructor calls start(); the destructor calls stop() and joins.
 *   - start()/stop() are idempotent and may be called explicitly (tests).
 */
class Localization {
public:
    Localization(std::shared_ptr<libstp::odometry::IOdometry> odometry,
                 LocalizationConfig config = {},
                 std::optional<libstp::map::WorldMap> tableMap = std::nullopt);
    ~Localization();

    Localization(const Localization&) = delete;
    Localization& operator=(const Localization&) = delete;
    Localization(Localization&&) = delete;
    Localization& operator=(Localization&&) = delete;

    /// Snapshot of the current world pose. Thread-safe.
    [[nodiscard]] libstp::foundation::Pose getPose() const;

    /// Apply a sigma-weighted pose observation. Infinite-sigma axes are ignored.
    void observe(const Observation& obs);

    /// Idempotent. The constructor already calls this; tests may re-call.
    void start();

    /// Idempotent. Joins the worker thread. The destructor calls this.
    void stop();

private:
    struct Particle {
        libstp::foundation::Pose pose{};
        double weight{1.0};
    };

    void initializeLocked(const libstp::foundation::Pose& anchorPose);
    void initializeParticlesLocked(const libstp::foundation::Pose& anchorPose,
                                   const Eigen::Vector3d& sigma);
    void predictLocked(const libstp::foundation::Pose& odomDelta);
    void integrateObservationLocked(const Observation& obs);
    [[nodiscard]] double computeParticleLogWeight(const Particle& particle,
                                                  const Observation& obs) const;
    void normalizeWeightsLocked();
    void resampleLocked();
    void updateEstimateLocked();
    void tickLoop(std::stop_token stop);

    std::shared_ptr<libstp::odometry::IOdometry> m_odometry;
    LocalizationConfig m_config;

    mutable std::mutex m_mutex;
    libstp::foundation::Pose m_worldPose{};
    libstp::foundation::Pose m_lastOdom{};
    std::vector<Particle> m_particles;
    bool m_initialized{false};
    std::mt19937 m_rng;
    std::optional<libstp::map::WorldMap> m_tableMap;

    // Background worker routed through ThreadManager: dropping this handle
    // requests stop on the std::jthread and joins it. The destructor relies
    // on that RAII contract; ``stop()`` calls it explicitly to keep the
    // start()/stop() API idempotent.
    libstp::threading::ThreadManager::DaemonHandle m_daemon;
};

}  // namespace libstp::localization
