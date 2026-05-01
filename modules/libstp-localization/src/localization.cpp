#include "localization/localization.hpp"

#include "odometry/angle_utils.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace libstp::localization {

namespace {

inline float wrapAnglef(float angle_rad) {
    return static_cast<float>(libstp::odometry::wrapAngle(static_cast<double>(angle_rad)));
}

}  // namespace

Localization::Localization(std::shared_ptr<libstp::odometry::IOdometry> odometry,
                           LocalizationConfig config)
    : m_odometry(std::move(odometry)), m_config(config) {
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

    if (std::isfinite(obs.sigma.x())) {
        m_worldPose.position.x() = obs.pose.position.x();
    }
    if (std::isfinite(obs.sigma.y())) {
        m_worldPose.position.y() = obs.pose.position.y();
    }
    if (std::isfinite(obs.sigma.z())) {
        m_worldPose.heading = wrapAnglef(obs.pose.heading);
    }

    // Rebase the odom reference so the next tick measures delta from "now",
    // preventing the snap from being undone by a stale delta.
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
                m_worldPose = cur;
                m_initialized = true;
            } else {
                const float dx = cur.position.x() - m_lastOdom.position.x();
                const float dy = cur.position.y() - m_lastOdom.position.y();
                const float dth = wrapAnglef(cur.heading - m_lastOdom.heading);

                m_worldPose.position.x() += dx;
                m_worldPose.position.y() += dy;
                m_worldPose.heading = wrapAnglef(m_worldPose.heading + dth);

                m_lastOdom = cur;
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
