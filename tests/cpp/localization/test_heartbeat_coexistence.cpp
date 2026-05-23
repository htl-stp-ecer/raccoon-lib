// Reproducers for the symptom "heartbeat stops once Localization is running,
// hardware watchdog fires, robot stops moving."
//
// The actual production heartbeat lives in libstp-hal's bindings and publishes
// over LCM, which we can't exercise from a unit test. Instead we register a
// **heartbeat-shaped surrogate** daemon (fixed-rate, atomic counter, exits on
// stop_token) and assert it keeps ticking under the same load patterns that
// the production code produces: long-lived Localization instance, observe()
// spam, many Localization create/destroy cycles, churn of unrelated scoped
// daemons.
//
// If any of these tests start failing, the *threading substrate* under the
// heartbeat is broken. If they keep passing while the real robot still drops
// heartbeats, the problem lives elsewhere (most likely concurrent LCM
// transport without internal locking).

#include <gtest/gtest.h>

#include "foundation/types.hpp"
#include "hal/odometry.hpp"
#include "localization/localization.hpp"
#include "threading/thread_manager.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using libstp::foundation::Pose;
using libstp::localization::Localization;
using libstp::localization::LocalizationConfig;
using libstp::localization::Observation;
using libstp::odometry::IOdometry;
using libstp::threading::ThreadManager;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

namespace {

class FakeOdometry : public IOdometry {
public:
    void update(double /*dt*/) override {}
    [[nodiscard]] Pose getPose() const override {
        std::lock_guard<std::mutex> lk(m_mutex);
        return m_pose;
    }
    [[nodiscard]] libstp::odometry::DistanceFromOrigin getDistanceFromOrigin() const override {
        return {0.0, 0.0, 0.0};
    }
    [[nodiscard]] double getHeading() const override {
        std::lock_guard<std::mutex> lk(m_mutex);
        return static_cast<double>(m_pose.heading);
    }
    [[nodiscard]] double getHeadingError(double target) const override {
        std::lock_guard<std::mutex> lk(m_mutex);
        return target - static_cast<double>(m_pose.heading);
    }
    [[nodiscard]] double getAbsoluteHeading() const override {
        std::lock_guard<std::mutex> lk(m_mutex);
        return static_cast<double>(m_pose.heading);
    }
    [[nodiscard]] double getPathLength() const override { return 0.0; }
    void reset() override {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_pose = Pose{};
    }
    void setPose(float x, float y, float heading) {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_pose.position.x() = x;
        m_pose.position.y() = y;
        m_pose.heading = heading;
    }

private:
    mutable std::mutex m_mutex;
    Pose m_pose{};
};

// Production heartbeat interval; we keep the surrogate at the same cadence
// because the symptom we're hunting is rate-sensitive.
constexpr auto kHeartbeatInterval = milliseconds(100);

// Register a heartbeat-shaped daemon. The returned shared_ptr<atomic<int>>
// tracks tick count from the test thread. Scoped (not persistent) so the
// daemon is torn down at end of the test even if the assertion fails.
struct HeartbeatProbe {
    std::shared_ptr<std::atomic<int>> ticks =
        std::make_shared<std::atomic<int>>(0);
    ThreadManager::DaemonHandle handle;
};

HeartbeatProbe spawnHeartbeatProbe(const char* name) {
    HeartbeatProbe probe;
    auto ticks = probe.ticks;
    probe.handle = ThreadManager::instance().add_daemon(
        name, [ticks](std::stop_token stop) {
            while (!stop.stop_requested()) {
                ticks->fetch_add(1, std::memory_order_relaxed);
                std::this_thread::sleep_for(kHeartbeatInterval);
            }
        });
    return probe;
}

int waitForTicksAtLeast(const HeartbeatProbe& probe, int target, milliseconds timeout) {
    const auto deadline = steady_clock::now() + timeout;
    while (steady_clock::now() < deadline) {
        if (probe.ticks->load(std::memory_order_relaxed) >= target) {
            break;
        }
        std::this_thread::sleep_for(milliseconds(5));
    }
    return probe.ticks->load(std::memory_order_relaxed);
}

LocalizationConfig fastConfig() {
    LocalizationConfig cfg;
    cfg.tick_period_ms = 5;
    cfg.particle_count = 128;
    cfg.rng_seed = 0xBEE7;
    return cfg;
}

}  // namespace

// Baseline: in isolation, a heartbeat-shaped daemon ticks ~10×/sec.
TEST(HeartbeatCoexistence, BaselineTicksAtFixedRate) {
    auto probe = spawnHeartbeatProbe("hb-baseline");
    const int ticks = waitForTicksAtLeast(probe, 4, milliseconds(700));
    EXPECT_GE(ticks, 4) << "Surrogate should fire ~7 times in 700 ms; got " << ticks;
}

// The headline scenario: Localization is up and running, heartbeat must keep
// ticking. If this regresses, the user sees the production symptom.
TEST(HeartbeatCoexistence, HeartbeatTicksWhileLocalizationRuns) {
    auto probe = spawnHeartbeatProbe("hb-with-loc");
    auto odom = std::make_shared<FakeOdometry>();
    Localization loc(odom, fastConfig());

    // Walk the odometry a bit so the localization tick loop has actual work
    // (predict + update) and isn't just idling on equal samples.
    for (int i = 0; i < 20; ++i) {
        odom->setPose(0.01f * static_cast<float>(i), 0.0f, 0.0f);
        std::this_thread::sleep_for(milliseconds(30));
    }

    const int ticks = probe.ticks->load(std::memory_order_relaxed);
    EXPECT_GE(ticks, 4)
        << "Localization tick loop must not starve a 100 ms-cadence daemon; "
        << "got " << ticks << " heartbeats in ~600 ms.";
}

// observe() takes the Localization mutex. Spamming it from a tester thread
// catches the case where mutex hold times bleed into other ThreadManager
// daemons (we don't expect that — observe() does not touch ThreadManager —
// but the test guards against future regressions).
TEST(HeartbeatCoexistence, HeartbeatSurvivesObservationStorm) {
    auto probe = spawnHeartbeatProbe("hb-observe-storm");
    auto odom = std::make_shared<FakeOdometry>();
    Localization loc(odom, fastConfig());

    Observation obs;
    obs.pose.position.x() = 0.10f;
    obs.sigma = Eigen::Vector3d{0.05, 0.05, 0.05};

    const auto deadline = steady_clock::now() + milliseconds(700);
    int observations = 0;
    while (steady_clock::now() < deadline) {
        loc.observe(obs);
        ++observations;
        std::this_thread::sleep_for(milliseconds(5));
    }

    const int ticks = probe.ticks->load(std::memory_order_relaxed);
    EXPECT_GT(observations, 50) << "Tester thread should have fired many observations";
    EXPECT_GE(ticks, 4) << "Got " << ticks << " heartbeats during " << observations
                       << " observations.";
}

// Multiple Localization instances simulate a poorly-cleaned-up test or a
// future multi-robot scenario. Heartbeat must still tick.
TEST(HeartbeatCoexistence, HeartbeatSurvivesMultipleLocalizationInstances) {
    auto probe = spawnHeartbeatProbe("hb-multi-loc");

    std::vector<std::shared_ptr<FakeOdometry>> odoms;
    std::vector<std::unique_ptr<Localization>> locs;
    for (int i = 0; i < 5; ++i) {
        auto odom = std::make_shared<FakeOdometry>();
        odoms.push_back(odom);
        locs.push_back(std::make_unique<Localization>(odom, fastConfig()));
    }

    const int ticks = waitForTicksAtLeast(probe, 4, milliseconds(700));
    EXPECT_GE(ticks, 4) << "Got " << ticks << " heartbeats with 5 concurrent Localizations.";
}

// Stress: create + destroy 50 Localization instances back-to-back. Catches
// thread-leak issues that would slowly degrade scheduling fairness, and
// double-acquire bugs in the ThreadManager registry.
TEST(HeartbeatCoexistence, HeartbeatSurvivesLocalizationChurn) {
    auto probe = spawnHeartbeatProbe("hb-churn");

    const int loc_count_before = static_cast<int>(ThreadManager::instance().daemon_count());

    for (int i = 0; i < 50; ++i) {
        auto odom = std::make_shared<FakeOdometry>();
        Localization loc(odom, fastConfig());
        std::this_thread::sleep_for(milliseconds(12));
    }

    const int ticks = probe.ticks->load(std::memory_order_relaxed);
    EXPECT_GE(ticks, 4) << "Got " << ticks << " heartbeats during 50 Localization churns.";

    const int loc_count_after = static_cast<int>(ThreadManager::instance().daemon_count());
    EXPECT_EQ(loc_count_after, loc_count_before)
        << "Localization daemons leaked: before=" << loc_count_before
        << " after=" << loc_count_after;
}

// A persistent (heartbeat-API) daemon stays registered until ThreadManager
// shutdown. Verify that registering one then starting/stopping Localization
// doesn't accidentally deregister it. We don't actually call
// add_persistent_daemon here (those leak by design until process exit and
// would pollute the singleton across test runs) — instead we lean on the
// scoped variant and check the lifecycle invariant.
TEST(HeartbeatCoexistence, ScopedDaemonOutlivesLocalizationLifecycle) {
    auto probe = spawnHeartbeatProbe("hb-outlives-loc");
    {
        auto odom = std::make_shared<FakeOdometry>();
        Localization loc(odom, fastConfig());
        std::this_thread::sleep_for(milliseconds(150));
    }  // Localization destructor fires here.

    // Heartbeat must continue ticking after Localization is gone.
    const int ticks_at_destroy = probe.ticks->load(std::memory_order_relaxed);
    std::this_thread::sleep_for(milliseconds(300));
    const int ticks_after = probe.ticks->load(std::memory_order_relaxed);
    EXPECT_GT(ticks_after, ticks_at_destroy)
        << "Heartbeat stopped ticking after Localization destruction. "
        << "Before=" << ticks_at_destroy << " after=" << ticks_after;
}
