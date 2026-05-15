// Auto-tick tests. Uses a freezable fake clock so behavior is deterministic.

#include <gtest/gtest.h>

#include "core/MockPlatform.hpp"
#include "hal/Platform.hpp"
#include "odometry/odometry.hpp"
#include "libstp/sim/SimWorld.hpp"

#include <cmath>
#include <memory>

using namespace libstp::sim;
namespace mock = platform::mock::core;

namespace
{
    struct FakeClock
    {
        double now{0.0};
        double operator()() const { return now; }
    };

    RobotConfig robot()
    {
        RobotConfig r{};
        r.widthCm = 18.0f;
        r.lengthCm = 18.0f;
        r.wheelRadiusM = 0.03f;
        r.trackWidthM = 0.15f;
        return r;
    }

    SimMotorMap motors()
    {
        SimMotorMap m{};
        m.leftPort = 0;
        m.rightPort = 1;
        m.maxWheelVelocityRadS = 30.0f;
        m.motorTimeConstantSec = 0.001f;  // near-instant response for clean math
        return m;
    }

    std::shared_ptr<libstp::odometry::IOdometry> makeOdometry()
    {
        return libstp::hal::platform::Platform::createOdometry(nullptr);
    }
}

class AutoTickTest : public ::testing::Test
{
protected:
    FakeClock clock{};

    void SetUp() override
    {
        auto& p = mock::MockPlatform::instance();
        p.detachSim();
        p.setClockSource([this] { return clock(); });

        WorldMap map;
        map.setTable(400.0f, 300.0f);
        p.configureSim(robot(), motors(), std::move(map), Pose2D{50.0f, 50.0f, 0.0f});
    }

    void TearDown() override
    {
        auto& p = mock::MockPlatform::instance();
        p.setAutoTickEnabled(false);
        p.detachSim();
        p.setClockSource(nullptr);  // restore default
    }
};

TEST_F(AutoTickTest, DisabledByDefault)
{
    auto& p = mock::MockPlatform::instance();
    EXPECT_FALSE(p.isAutoTickEnabled());

    // Even with the clock advancing, pose shouldn't move because no one ticks.
    p.setMotor(0, mock::MotorDir::CW, 400);
    p.setMotor(1, mock::MotorDir::CW, 400);

    clock.now = 1.0;
    auto odom = makeOdometry();
    const auto pose = odom->getPose();
    // Odometry reports pose relative to last reset; configure sets origin to
    // start pose, so a fresh sim with no ticks reads (0, 0, 0).
    EXPECT_NEAR(pose.position.x(), 0.0f, 1e-4f);
}

TEST_F(AutoTickTest, EnabledAdvancesOnReadOdometry)
{
    auto& p = mock::MockPlatform::instance();
    p.setAutoTickMaxStep(1.0f);  // allow large chunks for this test
    p.setAutoTickEnabled(true);

    p.setMotor(0, mock::MotorDir::CW, 400);
    p.setMotor(1, mock::MotorDir::CW, 400);

    // Advance the clock by 1 second, then read odometry — auto-tick should
    // integrate the motion.
    clock.now = 1.0;
    auto odom = makeOdometry();
    const auto pose = odom->getPose();
    // Relative-to-start: 0 + ≈ 0.9 m/s × 1 s = 0.90 m (τ is ~1 ms).
    EXPECT_GT(pose.position.x(), 0.85f);
    EXPECT_LT(pose.position.x(), 0.95f);
}

TEST_F(AutoTickTest, CapsStepToMax)
{
    auto& p = mock::MockPlatform::instance();
    p.setAutoTickMaxStep(0.05f);  // 50 ms cap (default)
    p.setAutoTickEnabled(true);

    p.setMotor(0, mock::MotorDir::CW, 400);
    p.setMotor(1, mock::MotorDir::CW, 400);

    // Clock jumps by 10 s, but a single auto-tick caps at 50 ms.
    clock.now = 10.0;
    auto odom = makeOdometry();
    const auto pose = odom->getPose();
    // 50 ms at ~0.9 m/s ≈ 0.045 m advance from origin.
    EXPECT_NEAR(pose.position.x(), 0.045f, 0.01f);
}

TEST_F(AutoTickTest, AccumulatesAcrossMultipleReads)
{
    auto& p = mock::MockPlatform::instance();
    p.setAutoTickMaxStep(0.05f);
    p.setAutoTickEnabled(true);

    p.setMotor(0, mock::MotorDir::CW, 400);
    p.setMotor(1, mock::MotorDir::CW, 400);

    auto odom = makeOdometry();

    // Tick the clock in 40 ms increments (under the 50 ms cap) 25 times → 1 s.
    for (int i = 0; i < 25; ++i)
    {
        clock.now += 0.04;
        (void)odom->getPose();
    }
    const auto pose = odom->getPose();
    // Should have traveled roughly the full 0.9 m over 1 s.
    EXPECT_GT(pose.position.x(), 0.80f);
    EXPECT_LT(pose.position.x(), 0.95f);
}

TEST_F(AutoTickTest, FrozenClockProducesNoAdvance)
{
    auto& p = mock::MockPlatform::instance();
    p.setAutoTickEnabled(true);

    p.setMotor(0, mock::MotorDir::CW, 400);
    p.setMotor(1, mock::MotorDir::CW, 400);

    // Clock stays put — multiple reads must all report the same pose.
    auto odom = makeOdometry();
    const auto a = odom->getPose();
    const auto b = odom->getPose();
    const auto c = odom->getPose();
    EXPECT_FLOAT_EQ(a.position.x(), b.position.x());
    EXPECT_FLOAT_EQ(b.position.x(), c.position.x());
}
