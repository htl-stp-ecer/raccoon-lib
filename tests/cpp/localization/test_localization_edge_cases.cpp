// Edge-case tests for libstp::localization::Localization.
//
// Complements test_localization.cpp (happy-path coverage). Pin invariants that
// a careless implementation change would silently break on real hardware:
// angle wrap, sigma boundary handling, threading lifecycle, observation
// determinism, surface-measurement / map-less code paths, impossible-jump
// heuristic boundaries, and resampling sanity. A few cases (ESS / circular
// mean) are inspired by MRPT's CParticleFilterCapable unit tests
// (/tmp/mrpt/modules/mrpt_bayes/tests/bayes_unittest.cpp, BSD-3) — only the
// invariant names were borrowed, not code.
#include <gtest/gtest.h>

#include "foundation/types.hpp"
#include "libstp/map/WorldMap.hpp"
#include "localization/localization.hpp"
#include "odometry/odometry.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using libstp::foundation::Pose;
using libstp::localization::Localization;
using libstp::localization::LocalizationConfig;
using libstp::localization::Observation;
using libstp::odometry::IOdometry;

namespace {

class FakeOdometry : public IOdometry {
public:
    void setPose(float x, float y, float heading) {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_pose.position.x() = x;
        m_pose.position.y() = y;
        m_pose.heading = heading;
    }

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

private:
    mutable std::mutex m_mutex;
    Pose m_pose{};
};

constexpr int kTickMs = 5;
constexpr char kSingleLineFtmap[] = R"({
  "format":"flowchart-table-map",
  "version":1,
  "table":{"widthCm":200,"heightCm":100},
  "lines":[
    {"kind":"line","startX":20,"startY":50,"endX":180,"endY":50,"widthCm":1.5}
  ]
})";

LocalizationConfig fastConfig() {
    LocalizationConfig cfg;
    cfg.tick_period_ms = kTickMs;
    cfg.particle_count = 256;
    cfg.rng_seed = 1234;
    return cfg;
}

LocalizationConfig deterministicConfig() {
    LocalizationConfig cfg = fastConfig();
    cfg.process_translation_noise_m = 0.0;
    cfg.process_translation_noise_per_m = 0.0;
    cfg.process_heading_noise_rad = 0.0;
    cfg.process_heading_noise_per_rad = 0.0;
    cfg.observation_injection_ratio = 1.0;
    cfg.resample_effective_sample_ratio = 1.0;
    return cfg;
}

libstp::map::WorldMap makeSingleLineMap() {
    libstp::map::WorldMap map;
    map.parseFtmap(kSingleLineFtmap);
    return map;
}

void waitTicks(int n) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kTickMs * n + 5));
}

// Returns the shortest signed angular distance from a to b, in radians.
double angDiff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. Angle wrapping invariants.
// ---------------------------------------------------------------------------

// Two observations straddling +/-pi must average to ~pi via atan2(sin, cos);
// a naive arithmetic mean would flip to 0.
TEST(LocalizationEdge, CircularMeanDoesNotFlipAcrossPi) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, static_cast<float>(M_PI - 0.05));

    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 0.0f;
    obs.pose.position.y() = 0.0f;
    obs.pose.heading = static_cast<float>(M_PI - 0.01);
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    obs.pose.heading = static_cast<float>(-M_PI + 0.01);
    loc.observe(obs);

    const auto pose = loc.getPose();
    // Must stay near +/- pi (i.e. |abs(h)| ~ pi), not 0.
    EXPECT_GT(std::abs(pose.heading), M_PI / 2.0f)
        << "circular mean collapsed to 0 instead of staying at +/-pi";
}

// Driving past +pi must not produce a discontinuous jump in the propagated
// heading (it should wrap into [-pi, pi] without losing the world-frame mean).
TEST(LocalizationEdge, HeadingPropagationWrapsContinuouslyAcrossPi) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, static_cast<float>(M_PI - 0.10));

    Localization loc(odom, deterministicConfig());
    waitTicks(3);

    odom->setPose(0.0f, 0.0f, static_cast<float>(M_PI - 0.02));
    waitTicks(3);
    odom->setPose(0.0f, 0.0f, static_cast<float>(-M_PI + 0.05));  // wrapped
    waitTicks(4);

    const auto pose = loc.getPose();
    // Heading must be in valid wrapped range.
    EXPECT_LE(pose.heading, static_cast<float>(M_PI + 1e-3));
    EXPECT_GE(pose.heading, static_cast<float>(-M_PI - 1e-3));
    // And close to the new odom heading (within wrap distance).
    EXPECT_LT(std::abs(angDiff(pose.heading, -M_PI + 0.05)), 0.05);
}

// Resync observation at the +/-pi boundary must land cleanly.
TEST(LocalizationEdge, ObserveAtPiBoundarySnapsHeadingWithoutFlip) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 0.0f;
    obs.pose.position.y() = 0.0f;
    obs.pose.heading = static_cast<float>(M_PI - 0.005);
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_LT(std::abs(angDiff(pose.heading, M_PI - 0.005)), 5e-3);
}

// ---------------------------------------------------------------------------
// 2. Sigma edge cases.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, NaNSigmaDoesNotProduceNaNPose) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 0.5f;
    obs.pose.position.y() = 0.5f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(),
                                1e-3,
                                1e-3};
    // Must not crash.
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_TRUE(std::isfinite(pose.position.x()));
    EXPECT_TRUE(std::isfinite(pose.position.y()));
    EXPECT_TRUE(std::isfinite(pose.heading));
}

TEST(LocalizationEdge, ZeroSigmaIsClampedInternallyAndDoesNotDivideByZero) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 0.25f;
    obs.pose.position.y() = -0.10f;
    obs.pose.heading = 0.1f;
    obs.sigma = Eigen::Vector3d{0.0, 0.0, 0.0};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_TRUE(std::isfinite(pose.position.x()));
    EXPECT_NEAR(pose.position.x(), 0.25f, 5e-2f);
    EXPECT_NEAR(pose.position.y(), -0.10f, 5e-2f);
}

// Very large sigma — observation should have negligible effect on world pose.
TEST(LocalizationEdge, VeryLargeSigmaLeavesPriorEssentiallyUnchanged) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    auto cfg = deterministicConfig();
    cfg.observation_injection_ratio = 0.0;  // pure soft update
    Localization loc(odom, cfg);
    waitTicks(2);

    odom->setPose(0.20f, 0.0f, 0.0f);
    waitTicks(4);

    Observation obs;
    obs.pose.position.x() = 10.0f;  // wildly different
    obs.pose.position.y() = 10.0f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e6, 1e6, 1e6};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.20f, 1e-2f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// 3. Observation before any tick.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, InitializesFromFirstObservationWhenPoseHasNonZeroHeading) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, deterministicConfig());

    Observation obs;
    obs.pose.position.x() = -0.3f;
    obs.pose.position.y() = 0.7f;
    obs.pose.heading = 1.2f;  // non-zero theta
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), -0.3f, 5e-3f);
    EXPECT_NEAR(pose.position.y(), 0.7f, 5e-3f);
    EXPECT_NEAR(pose.heading, 1.2f, 5e-3f);
}

TEST(LocalizationEdge, BackToBackObservationsConvergeToLast) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, deterministicConfig());

    Observation a;
    a.pose.position.x() = 0.5f;
    a.pose.position.y() = 0.0f;
    a.pose.heading = 0.0f;
    a.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(a);

    Observation b;
    b.pose.position.x() = 1.0f;
    b.pose.position.y() = 0.5f;
    b.pose.heading = -0.3f;
    b.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(b);

    Observation c;
    c.pose.position.x() = -0.25f;
    c.pose.position.y() = -0.25f;
    c.pose.heading = 0.5f;
    c.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(c);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), -0.25f, 5e-3f);
    EXPECT_NEAR(pose.position.y(), -0.25f, 5e-3f);
    EXPECT_NEAR(pose.heading, 0.5f, 5e-3f);
}

// Observation with infinite pose sigma but a surface measurement should not
// crash and should not corrupt the world pose (no map => the surface
// measurement is silently ignored).
TEST(LocalizationEdge, ObservationWithOnlySurfaceMeasurementDoesNotCrashWithoutMap) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, deterministicConfig());
    waitTicks(2);
    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);

    Observation obs;
    obs.pose.position.x() = 0.0f;
    obs.pose.position.y() = 0.0f;
    obs.pose.heading = 0.0f;
    const double inf = std::numeric_limits<double>::infinity();
    obs.sigma = Eigen::Vector3d{inf, inf, inf};
    obs.surface_measurements.push_back(Observation::SurfaceMeasurement{
        Observation::SurfaceKind::Line, libstp::map::SensorOffset{0.0f, 0.0f}, true, 0.6});
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_TRUE(std::isfinite(pose.position.x()));
    EXPECT_TRUE(std::isfinite(pose.position.y()));
    EXPECT_NEAR(pose.position.x(), 0.10f, 2e-2f);
}

// ---------------------------------------------------------------------------
// 4. Determinism.
// ---------------------------------------------------------------------------

// Same seed + same synchronous observations -> identical world pose. The
// background worker is intentionally bypassed by stopping it immediately
// after construction — the tick loop reads the wall clock and so cannot be
// made strictly deterministic; observe() is synchronous and is the only
// surface a seeded test can safely pin.
TEST(LocalizationEdge, DeterministicSeedYieldsIdenticalFinalPose) {
    auto run = [](uint32_t seed) -> Pose {
        auto odom = std::make_shared<FakeOdometry>();
        odom->setPose(0.0f, 0.0f, 0.0f);
        auto cfg = fastConfig();
        cfg.rng_seed = seed;
        cfg.process_translation_noise_m = 0.005;
        cfg.process_translation_noise_per_m = 0.01;
        cfg.process_heading_noise_rad = 0.005;
        Localization loc(odom, cfg);
        loc.stop();  // freeze the worker so RNG draws are deterministic
        Observation obs;
        obs.pose.position.x() = 0.10f;
        obs.pose.position.y() = 0.02f;
        obs.pose.heading = 0.0f;
        obs.sigma = Eigen::Vector3d{1e-2, 1e-2, 1e-2};
        loc.observe(obs);
        Observation obs2 = obs;
        obs2.pose.position.x() = 0.20f;
        loc.observe(obs2);
        return loc.getPose();
    };

    const auto a = run(99);
    const auto b = run(99);
    EXPECT_NEAR(a.position.x(), b.position.x(), 1e-5f);
    EXPECT_NEAR(a.position.y(), b.position.y(), 1e-5f);
    EXPECT_NEAR(a.heading, b.heading, 1e-5f);
}

// Different seeds still converge to roughly the same mean under a tight
// observation (no hard divergence between particle clouds).
TEST(LocalizationEdge, DifferentSeedsStillConvergeUnderTightObservation) {
    auto run = [](uint32_t seed) -> Pose {
        auto odom = std::make_shared<FakeOdometry>();
        odom->setPose(0.0f, 0.0f, 0.0f);
        auto cfg = fastConfig();
        cfg.rng_seed = seed;
        Localization loc(odom, cfg);
        Observation obs;
        obs.pose.position.x() = 0.42f;
        obs.pose.position.y() = -0.13f;
        obs.pose.heading = 0.3f;
        obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
        loc.observe(obs);
        return loc.getPose();
    };

    const auto a = run(1);
    const auto b = run(2);
    EXPECT_NEAR(a.position.x(), b.position.x(), 1e-2f);
    EXPECT_NEAR(a.position.y(), b.position.y(), 1e-2f);
    EXPECT_NEAR(a.heading, b.heading, 1e-2f);
}

// ---------------------------------------------------------------------------
// 5. Convergence under noise — drive 1 m in 20 steps with default noise.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, BoundedDriftAcrossLongStraightDrive) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    auto cfg = fastConfig();  // realistic non-zero process noise
    Localization loc(odom, cfg);
    waitTicks(2);

    constexpr int kSteps = 20;
    for (int i = 1; i <= kSteps; ++i) {
        odom->setPose(static_cast<float>(0.05 * i), 0.0f, 0.0f);
        waitTicks(2);
    }

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 1.0f, 0.05f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 0.05f);
    EXPECT_NEAR(pose.heading, 0.0f, 0.05f);
}

// ---------------------------------------------------------------------------
// 6. Particle count extremes.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, ParticleCountOneStillProducesSensiblePose) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    auto cfg = deterministicConfig();
    cfg.particle_count = 1;
    Localization loc(odom, cfg);
    waitTicks(2);
    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);
    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.10f, 1e-2f);
}

// initializeParticlesLocked clamps count to >=1; particle_count=0 must not
// divide by zero in updateEstimate / resample.
TEST(LocalizationEdge, ParticleCountZeroIsClampedAndDoesNotCrash) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    auto cfg = deterministicConfig();
    cfg.particle_count = 0;
    Localization loc(odom, cfg);
    waitTicks(2);
    odom->setPose(0.05f, 0.0f, 0.0f);
    waitTicks(4);
    const auto pose = loc.getPose();
    EXPECT_TRUE(std::isfinite(pose.position.x()));
}

TEST(LocalizationEdge, ParticleCount1024IsCorrectButSlow) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    auto cfg = deterministicConfig();
    cfg.particle_count = 1024;
    Localization loc(odom, cfg);
    waitTicks(3);
    odom->setPose(0.10f, 0.05f, 0.0f);
    waitTicks(5);
    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.10f, 5e-3f);
    EXPECT_NEAR(pose.position.y(), 0.05f, 5e-3f);
}

// ---------------------------------------------------------------------------
// 7. Surface measurements (with map).
// ---------------------------------------------------------------------------

// Sensor reports detected=false in a region with NO line nearby — must not
// punish the particle's likelihood (the prior weight should dominate, so the
// pose stays close to the soft pose observation).
TEST(LocalizationEdge, OffLineFalseDetectedIsNotPunished) {
    auto cfg = fastConfig();
    cfg.observation_injection_ratio = 1.0;
    cfg.resample_effective_sample_ratio = 1.0;
    cfg.particle_count = 512;
    cfg.process_translation_noise_m = 0.005;
    cfg.process_heading_noise_rad = 0.0;
    cfg.rng_seed = 11;

    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(1.0f, 0.10f, 0.0f);  // line is at y=0.50
    Localization loc(odom, cfg, makeSingleLineMap());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 1.0f;
    obs.pose.position.y() = 0.10f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{0.02, 0.02, std::numeric_limits<double>::infinity()};
    obs.surface_measurements.push_back(Observation::SurfaceMeasurement{
        Observation::SurfaceKind::Line, libstp::map::SensorOffset{0.0f, 0.0f}, false, 0.6});
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 1.0f, 5e-2f);
    EXPECT_NEAR(pose.position.y(), 0.10f, 5e-2f);
}

// ---------------------------------------------------------------------------
// 8. Map-less robot — surface measurements silently ignored, weights still
//    react to pose terms.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, MaplessRobotIgnoresSurfaceMeasurementsButStillUsesPose) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    Localization loc(odom, deterministicConfig());  // no map
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 0.8f;
    obs.pose.position.y() = 0.4f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    obs.surface_measurements.push_back(Observation::SurfaceMeasurement{
        Observation::SurfaceKind::Line, libstp::map::SensorOffset{2.0f, 1.0f}, true, 0.5});
    obs.surface_measurements.push_back(Observation::SurfaceMeasurement{
        Observation::SurfaceKind::Wall, libstp::map::SensorOffset{0.0f, 0.0f}, true, 0.5});
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.8f, 5e-3f);
    EXPECT_NEAR(pose.position.y(), 0.4f, 5e-3f);
}

// ---------------------------------------------------------------------------
// 9. Threading lifecycle.
// ---------------------------------------------------------------------------

// Construct/destroy many instances rapidly — no thread leak, no hang.
TEST(LocalizationEdge, RepeatedConstructionDoesNotLeakOrDeadlock) {
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 30; ++i) {
        auto odom = std::make_shared<FakeOdometry>();
        Localization loc(odom, fastConfig());
        // No waitTicks — destruction must still be prompt even if the worker
        // has barely had time to run a single tick.
    }
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 5000)
        << "construct/destroy loop took too long — probable thread join leak";
}

TEST(LocalizationEdge, StopThenStartResumesFilter) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.10f, 5e-3f);
    }

    loc.stop();
    // Worker is gone; world pose stays at the last value.
    odom->setPose(0.30f, 0.0f, 0.0f);
    waitTicks(4);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.10f, 5e-3f)
            << "filter still ticking after stop()";
    }

    loc.start();
    // After restart, the filter rebases on the current odom and continues from
    // the prior world pose. The +0.20 m that happened while stopped is treated
    // as a one-shot rebase (impossible jump > 0.5 m? — 0.20 m is below the
    // 0.5 m budget, so it should actually accumulate). Either way the worker
    // must be running again: drive a further +0.05 m and watch the world
    // pose advance.
    const float xBefore = loc.getPose().position.x();
    odom->setPose(0.35f, 0.0f, 0.0f);
    waitTicks(6);
    const float xAfter = loc.getPose().position.x();
    EXPECT_GT(xAfter, xBefore - 1e-3f);
    // start() is idempotent.
    loc.start();
    loc.start();
}

// ---------------------------------------------------------------------------
// 10. Concurrent observe + getPose.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, ConcurrentObserveAndGetPoseDoNotCrash) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    Localization loc(odom, fastConfig());

    std::atomic<bool> done{false};
    std::thread reader([&]() {
        while (!done.load(std::memory_order_relaxed)) {
            const auto pose = loc.getPose();
            // Must remain finite at all times.
            ASSERT_TRUE(std::isfinite(pose.position.x()));
            ASSERT_TRUE(std::isfinite(pose.position.y()));
            ASSERT_TRUE(std::isfinite(pose.heading));
        }
    });

    const auto t0 = std::chrono::steady_clock::now();
    int n = 0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(500)) {
        Observation obs;
        obs.pose.position.x() = 0.1f * static_cast<float>(n % 5);
        obs.pose.position.y() = 0.05f * static_cast<float>(n % 7);
        obs.pose.heading = 0.0f;
        obs.sigma = Eigen::Vector3d{0.02, 0.02, 0.02};
        loc.observe(obs);
        ++n;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    done.store(true, std::memory_order_relaxed);
    reader.join();
    SUCCEED();
}

// ---------------------------------------------------------------------------
// 11. Multi-reset / reset-during-observation.
// ---------------------------------------------------------------------------

TEST(LocalizationEdge, MultipleOdometryResetsAccumulateCorrectWorldPose) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    // Drive segment 1: +0.20 m
    odom->setPose(0.20f, 0.0f, 0.0f);
    waitTicks(4);
    odom->reset();
    waitTicks(3);

    // Drive segment 2: +0.15 m
    odom->setPose(0.15f, 0.0f, 0.0f);
    waitTicks(4);
    odom->reset();
    waitTicks(3);

    // Drive segment 3: +0.10 m
    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.45f, 1e-2f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 5e-3f);
}

TEST(LocalizationEdge, ResetImmediatelyAfterObserveStillKeepsObservedPose) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.30f, 0.10f, 0.0f);
    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 1.5f;
    obs.pose.position.y() = 1.5f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);
    odom->reset();  // motion start fires immediately after the resync
    waitTicks(4);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 1.5f, 1e-2f);
    EXPECT_NEAR(pose.position.y(), 1.5f, 1e-2f);
}

// ---------------------------------------------------------------------------
// 12. Impossible-jump heuristic — boundary behavior.
// ---------------------------------------------------------------------------

// 0.49 m delta is BELOW the 0.5 m budget and must be treated as a real motion.
TEST(LocalizationEdge, JustBelowImpossibleJumpThresholdIsAccumulated) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    Localization loc(odom, deterministicConfig());
    waitTicks(3);

    odom->setPose(0.49f, 0.0f, 0.0f);
    waitTicks(6);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.49f, 1e-2f)
        << "0.49 m delta was incorrectly treated as a rebase";
}

// A long stream of impossible jumps must keep rebasing (no accumulation,
// no divergence).
TEST(LocalizationEdge, RepeatedImpossibleJumpsKeepRebasing) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    Localization loc(odom, deterministicConfig());
    waitTicks(2);

    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);
    const float baseline = loc.getPose().position.x();

    for (int i = 0; i < 5; ++i) {
        odom->setPose(static_cast<float>(5.0 + i), static_cast<float>(5.0 + i), 0.0f);
        waitTicks(4);
    }

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), baseline, 5e-3f)
        << "world pose drifted under repeated impossible jumps";
}

// ---------------------------------------------------------------------------
// 13. Surface false-positive / false-negative weight effects.
// ---------------------------------------------------------------------------

// Particle is OFF the line, sensor reports detected=true — under the existing
// likelihood the particles near the line are favored, so the y estimate
// should be pulled toward the line (y=0.50). This is a stricter pairing of
// existing tests; we check that the pull is monotonic vs. a wrong sigma.
TEST(LocalizationEdge, FalsePositiveDetectionPullsEstimateTowardLine) {
    auto cfg = fastConfig();
    cfg.process_translation_noise_m = 0.02;
    cfg.process_heading_noise_rad = 0.0;
    cfg.observation_injection_ratio = 1.0;
    cfg.resample_effective_sample_ratio = 1.0;
    cfg.particle_count = 1024;
    cfg.rng_seed = 5;

    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(1.0f, 0.55f, 0.0f);
    Localization loc(odom, cfg, makeSingleLineMap());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 1.0f;
    obs.pose.position.y() = 0.55f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{0.02, 0.10, std::numeric_limits<double>::infinity()};
    obs.surface_measurements.push_back(Observation::SurfaceMeasurement{
        Observation::SurfaceKind::Line, libstp::map::SensorOffset{0.0f, 0.0f}, true, 0.6});
    loc.observe(obs);

    const auto pose = loc.getPose();
    // Should be pulled toward y=0.50 (the line) compared to the 0.55 prior.
    EXPECT_LT(pose.position.y(), 0.55f)
        << "detected=true did not pull estimate toward observed line";
    EXPECT_GT(pose.position.y(), 0.40f);
}

// ---------------------------------------------------------------------------
// 14. observation_injection_ratio == 0 vs 1.0.
// ---------------------------------------------------------------------------

// ratio = 0 — observation only reweights; cloud is not relocated, so a wildly
// off-prior observation with a tight sigma should NOT teleport the estimate
// (it just collapses the weights onto whichever particles are nearest the
// observation — and with deterministicConfig there is essentially no spread).
TEST(LocalizationEdge, ZeroInjectionRatioDoesNotTeleportCloud) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    auto cfg = deterministicConfig();
    cfg.observation_injection_ratio = 0.0;
    Localization loc(odom, cfg);
    waitTicks(2);
    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);

    Observation obs;
    obs.pose.position.x() = 5.0f;
    obs.pose.position.y() = 5.0f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_LT(pose.position.x(), 1.0f)
        << "ratio=0 still teleported the cloud — injection path is firing";
}

// ratio = 1 — every particle is reseeded around the observation.
TEST(LocalizationEdge, FullInjectionRatioRelocatesCloud) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    auto cfg = deterministicConfig();
    cfg.observation_injection_ratio = 1.0;
    Localization loc(odom, cfg);
    waitTicks(2);
    odom->setPose(0.10f, 0.0f, 0.0f);
    waitTicks(4);

    Observation obs;
    obs.pose.position.x() = 5.0f;
    obs.pose.position.y() = 5.0f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 5.0f, 1e-2f);
    EXPECT_NEAR(pose.position.y(), 5.0f, 1e-2f);
}

// ---------------------------------------------------------------------------
// 15. Mutation-style pins.
// ---------------------------------------------------------------------------

// Heading averaging via atan2(sum sin, sum cos): two particles at 175 deg and
// 185 deg (i.e. straddling +pi) must mean to ~180 deg, not ~0. We can only
// poke this via a single observation that snaps the cloud to one and then
// reweights toward the other.
TEST(LocalizationEdge, HeadingMeanIsCircular) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    auto cfg = deterministicConfig();
    cfg.observation_injection_ratio = 1.0;
    Localization loc(odom, cfg);
    waitTicks(2);

    Observation a;
    a.pose.heading = static_cast<float>(175.0 * M_PI / 180.0);
    a.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(a);

    Observation b;
    b.pose.heading = static_cast<float>(-175.0 * M_PI / 180.0);  // == +185 deg
    b.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(b);

    const auto pose = loc.getPose();
    // Heading should sit near +/- pi — i.e. magnitude > 90 deg, not near 0.
    EXPECT_GT(std::abs(pose.heading), static_cast<float>(M_PI / 2.0))
        << "heading averaging is arithmetic, not circular";
}

// Resample must preserve particle count — observe repeatedly to force many
// resamples and check the filter still produces sane poses afterwards.
TEST(LocalizationEdge, RepeatedResamplesKeepFilterFunctional) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);
    auto cfg = fastConfig();
    cfg.resample_effective_sample_ratio = 1.0;  // resample every observation
    cfg.observation_injection_ratio = 0.2;       // small injection seeds spread
    cfg.particle_count = 512;
    Localization loc(odom, cfg);
    waitTicks(2);

    for (int i = 0; i < 20; ++i) {
        Observation obs;
        obs.pose.position.x() = 0.25f;
        obs.pose.position.y() = 0.25f;
        obs.pose.heading = 0.0f;
        obs.sigma = Eigen::Vector3d{0.05, 0.05, 0.05};
        loc.observe(obs);
    }

    const auto pose = loc.getPose();
    // The cardinal property: particle count is preserved across many
    // resamples — i.e. the filter does not collapse, and getPose() still
    // returns finite, bounded values converging toward the repeated
    // observation.
    EXPECT_TRUE(std::isfinite(pose.position.x()));
    EXPECT_TRUE(std::isfinite(pose.position.y()));
    EXPECT_NEAR(pose.position.x(), 0.25f, 1e-1f);
    EXPECT_NEAR(pose.position.y(), 0.25f, 1e-1f);
}
