#pragma once

#include "foundation/types.hpp"
#include "odometry/odometry.hpp"

#include <Eigen/Core>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace libstp::localization {

/**
 * @brief Soft-snap observation pushed by Resync-Steps.
 *
 * `sigma` is the measurement standard deviation per axis (x, y, theta).
 * In the Phase-2 pass-through service, an axis with finite sigma is hard-snapped;
 * an infinite sigma leaves that axis untouched. The proper soft snap (likelihood
 * weighting) lands when the particle filter replaces the pass-through in Phase 6.
 */
struct Observation {
    libstp::foundation::Pose pose{};
    Eigen::Vector3d sigma{1e-3, 1e-3, 1e-3};
};

struct LocalizationConfig {
    int tick_period_ms{10};  ///< 100 Hz default
};

/**
 * @brief Thread-safe pass-through localization service.
 *
 * Owns a background thread that polls the supplied IOdometry at a fixed rate,
 * accumulates the per-tick delta into a world-frame pose, and exposes that pose
 * via @ref getPose(). @ref observe() lets resync sites snap selected axes.
 *
 * Phase-2 contract: pass-through only. No filter, no covariance. The world frame
 * matches the odometry frame except for the discrete jumps introduced by
 * @ref observe(); after each observe call the next tick's delta is rebased so
 * the accumulated pose does not double-count.
 *
 * Lifecycle:
 *   - The constructor calls start(); the destructor calls stop() and joins.
 *   - start()/stop() are idempotent and may be called explicitly (tests).
 */
class Localization {
public:
    Localization(std::shared_ptr<libstp::odometry::IOdometry> odometry,
                 LocalizationConfig config = {});
    ~Localization();

    Localization(const Localization&) = delete;
    Localization& operator=(const Localization&) = delete;
    Localization(Localization&&) = delete;
    Localization& operator=(Localization&&) = delete;

    /// Snapshot of the current world pose. Thread-safe.
    [[nodiscard]] libstp::foundation::Pose getPose() const;

    /// Apply a soft observation (Phase 2: hard snap on finite-sigma axes).
    void observe(const Observation& obs);

    /// Idempotent. The constructor already calls this; tests may re-call.
    void start();

    /// Idempotent. Joins the worker thread. The destructor calls this.
    void stop();

private:
    void tickLoop();

    std::shared_ptr<libstp::odometry::IOdometry> m_odometry;
    LocalizationConfig m_config;

    mutable std::mutex m_mutex;
    libstp::foundation::Pose m_worldPose{};
    libstp::foundation::Pose m_lastOdom{};
    bool m_initialized{false};

    std::atomic<bool> m_running{false};
    std::thread m_thread;
};

}  // namespace libstp::localization
