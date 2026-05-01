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
