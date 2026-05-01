#include <gtest/gtest.h>

#include "foundation/types.hpp"
#include "localization/localization.hpp"
#include "odometry/odometry.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

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

// Small helper: tick period 5 ms in tests so we don't have to wait long.
constexpr int kTickMs = 5;

LocalizationConfig fastConfig() {
    return LocalizationConfig{kTickMs};
}

void waitTicks(int n) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kTickMs * n + 5));
}

}  // namespace

TEST(Localization, PropagatesPose) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(3);

    odom->setPose(0.1f, 0.0f, 0.0f);
    waitTicks(3);
    odom->setPose(0.2f, 0.0f, 0.0f);
    waitTicks(5);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.2f, 1e-4f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-4f);
    EXPECT_NEAR(pose.heading, 0.0f, 1e-4f);
}

TEST(Localization, ObserveSnapsAllAxes) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(2);

    Observation obs;
    obs.pose.position.x() = 1.5f;
    obs.pose.position.y() = 1.5f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    // Snap is immediate (synchronous under the lock).
    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 1.5f, 1e-4f);
    EXPECT_NEAR(pose.position.y(), 1.5f, 1e-4f);
    EXPECT_NEAR(pose.heading, 0.0f, 1e-4f);
}

TEST(Localization, ObserveDoesNotDoubleCount) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(2);

    // Accumulate some odom drift first.
    odom->setPose(0.30f, 0.0f, 0.0f);
    waitTicks(4);

    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.30f, 1e-4f);
    }

    // Snap to a known absolute pose.
    Observation obs;
    obs.pose.position.x() = 1.0f;
    obs.pose.position.y() = 0.0f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 1.0f, 1e-4f);
    }

    // Move odom further; world pose must advance by the new delta only,
    // not re-add the pre-snap accumulated 0.30 m.
    odom->setPose(0.50f, 0.0f, 0.0f);  // delta from rebase = +0.20 m
    waitTicks(5);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 1.20f, 1e-4f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-4f);
}

TEST(Localization, StopAndDestructorJoinPromptly) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    const auto t0 = std::chrono::steady_clock::now();
    {
        Localization loc(odom, fastConfig());
        waitTicks(2);
        loc.stop();
        // stop() is idempotent — calling again must not hang.
        loc.stop();
    }
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 200);
}

// Phase-2 headline promise: "Welt-Pose lebt über Motion-Grenzen hinweg."
// Every motion start() still calls odometry.reset() (cleanup is Phase 4);
// the pass-through must notice that the underlying odom snapped back to
// the origin and treat the jump as a rebase, not as a real -0.20 m delta.
TEST(Localization, WorldPoseSurvivesOdometryReset) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(2);

    // Simulate motion 1: odometry integrates to 0.20 m forward.
    odom->setPose(0.20f, 0.0f, 0.0f);
    waitTicks(4);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.20f, 1e-4f);
    }

    // Motion 2 starts — the motion's start() calls odometry.reset(). The
    // world pose must NOT collapse: localization owns the world frame.
    odom->reset();
    waitTicks(4);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.20f, 1e-3f)
            << "world pose collapsed after odometry.reset() — Phase-2 promise "
               "broken; localization needs reset detection";
    }

    // Motion 2 integrates another 0.20 m forward (odom counts from 0).
    odom->setPose(0.20f, 0.0f, 0.0f);
    waitTicks(4);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.40f, 1e-3f)
            << "world pose did not advance across the second motion";
    }
}

TEST(Localization, ObserveWithInfiniteSigmaLeavesAxisUntouched) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(2);

    odom->setPose(0.40f, 0.25f, 0.0f);
    waitTicks(4);

    Observation obs;
    obs.pose.position.x() = 9.99f;  // ignored
    obs.pose.position.y() = 1.00f;  // applied
    obs.pose.heading = 9.99f;       // ignored
    obs.sigma = Eigen::Vector3d{std::numeric_limits<double>::infinity(),
                                1e-3,
                                std::numeric_limits<double>::infinity()};
    loc.observe(obs);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.40f, 1e-4f);
    EXPECT_NEAR(pose.position.y(), 1.00f, 1e-4f);
    EXPECT_NEAR(pose.heading, 0.0f, 1e-4f);
}

// After observe() snaps the world pose, an odometry reset must still leave
// the snapped world pose intact — otherwise the very first motion that fires
// after a resync collapses everything we just measured.
TEST(Localization, WorldPoseSurvivesResetAfterObserve) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(2);

    // Drive a bit so the odometry frame is non-trivial.
    odom->setPose(0.30f, 0.10f, 0.0f);
    waitTicks(4);

    // Soft-snap the world pose to a known-good location (e.g. resync).
    Observation obs;
    obs.pose.position.x() = 1.5f;
    obs.pose.position.y() = 1.5f;
    obs.pose.heading = 0.0f;
    obs.sigma = Eigen::Vector3d{1e-3, 1e-3, 1e-3};
    loc.observe(obs);

    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 1.5f, 1e-4f);
        EXPECT_NEAR(pose.position.y(), 1.5f, 1e-4f);
    }

    // Next motion starts → odometry.reset(). World pose must hold at (1.5, 1.5).
    odom->reset();
    waitTicks(4);

    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 1.5f, 1e-3f)
        << "snapped world pose collapsed when motion reset the odometry";
    EXPECT_NEAR(pose.position.y(), 1.5f, 1e-3f);
    EXPECT_NEAR(pose.heading, 0.0f, 1e-3f);
}

// Defensive arm of the reset heuristic: any single-tick jump larger than
// ``kMaxPlausibleStep`` (0.5 m) is physically impossible for the platform
// and must be treated as a rebase rather than accumulated. This protects
// against unclean reset paths and any future odometry source whose reset
// does not land exactly on the origin.
TEST(Localization, ImpossibleJumpIsTreatedAsReset) {
    auto odom = std::make_shared<FakeOdometry>();
    odom->setPose(0.0f, 0.0f, 0.0f);

    Localization loc(odom, fastConfig());
    waitTicks(2);

    // Build up a normal world pose first.
    odom->setPose(0.20f, 0.0f, 0.0f);
    waitTicks(4);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.20f, 1e-4f);
    }

    // Teleport the odometry to (10, 10, 0) — far beyond the per-tick step
    // budget. World pose must NOT accumulate the +9.8 m delta.
    odom->setPose(10.0f, 10.0f, 0.0f);
    waitTicks(5);
    {
        const auto pose = loc.getPose();
        EXPECT_NEAR(pose.position.x(), 0.20f, 1e-3f)
            << "impossible jump was accumulated instead of rebased";
        EXPECT_NEAR(pose.position.y(), 0.0f, 1e-3f);
    }

    // From the rebase point, a small motion (10.0 → 10.05) must be picked
    // up normally — i.e. the rebase did not break the delta path.
    odom->setPose(10.05f, 10.0f, 0.0f);
    waitTicks(5);
    const auto pose = loc.getPose();
    EXPECT_NEAR(pose.position.x(), 0.25f, 1e-3f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-3f);
}
