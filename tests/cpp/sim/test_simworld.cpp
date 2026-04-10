#include <gtest/gtest.h>

#include "libstp/sim/SimWorld.hpp"

#include <cmath>

using namespace libstp::sim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    RobotConfig defaultRobot()
    {
        RobotConfig r{};
        r.widthCm = 18.0f;
        r.lengthCm = 18.0f;
        r.driveType = "differential";
        r.wheelRadiusM = 0.03f;      // 30 mm
        r.trackWidthM = 0.15f;        // 150 mm
        r.wheelbaseM = 0.15f;
        return r;
    }

    SimMotorMap defaultMotors()
    {
        SimMotorMap m{};
        m.leftPort = 0;
        m.rightPort = 1;
        m.leftInverted = false;
        m.rightInverted = false;
        m.maxWheelVelocityRadS = 30.0f;   // ≈ 0.9 m/s at this wheel radius
        m.motorTimeConstantSec = 0.05f;   // snappy
        return m;
    }

    /// Run the sim forward for `seconds`, with a fixed 10ms step.
    void run(SimWorld& sim, float seconds, float dt = 0.01f)
    {
        const int steps = static_cast<int>(std::round(seconds / dt));
        for (int i = 0; i < steps; ++i) sim.tick(dt);
    }
}

TEST(SimWorldTest, StationaryWithZeroCommand)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({50.0f, 50.0f, 0.0f});
    run(sim, 1.0f);
    const auto p = sim.pose();
    EXPECT_NEAR(p.x, 50.0f, 1e-4f);
    EXPECT_NEAR(p.y, 50.0f, 1e-4f);
    EXPECT_NEAR(p.theta, 0.0f, 1e-4f);
}

TEST(SimWorldTest, DrivesForwardAtExpectedVelocity)
{
    // 100% both wheels → v = r · ω_max = 0.03 · 30 = 0.9 m/s = 90 cm/s.
    // After 2 s, x should advance by ~180 cm (minus small lag from τ=50 ms).
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({0.0f, 0.0f, 0.0f});
    sim.setMotorCommand(0, 100);
    sim.setMotorCommand(1, 100);
    run(sim, 2.0f);

    const auto p = sim.pose();
    EXPECT_NEAR(p.y, 0.0f, 1e-3f);
    EXPECT_NEAR(p.theta, 0.0f, 1e-3f);
    // Expected ≈ 180 cm; initial ramp-up over τ costs about r·ω_max·τ ≈ 4.5 cm.
    EXPECT_NEAR(p.x, 180.0f - 4.5f, 1.0f);
}

TEST(SimWorldTest, DrivesBackwardWithNegativeCommand)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({100.0f, 50.0f, 0.0f});
    sim.setMotorCommand(0, -50);
    sim.setMotorCommand(1, -50);
    run(sim, 1.0f);

    const auto p = sim.pose();
    EXPECT_LT(p.x, 100.0f);
    EXPECT_NEAR(p.y, 50.0f, 1e-3f);
    EXPECT_NEAR(p.theta, 0.0f, 1e-3f);
}

TEST(SimWorldTest, TurnsInPlaceWithOpposingWheels)
{
    // Yaw rate = (r/W)·(ω_R − ω_L). With ω_L = +30 and ω_R = −30:
    // yaw_rate = 0.2·(−60) = −12 rad/s. Keep the run short so the raw rotation
    // stays in (−π, π] and doesn't wrap.
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({50.0f, 50.0f, 0.0f});
    sim.setMotorCommand(0, 100);   // left forward
    sim.setMotorCommand(1, -100);  // right backward
    run(sim, 0.2f);

    const auto p = sim.pose();
    EXPECT_NEAR(p.x, 50.0f, 0.5f);
    EXPECT_NEAR(p.y, 50.0f, 0.5f);
    // Expected ≈ −12·0.2 = −2.4 rad (minus a small ramp-in cost).
    EXPECT_LT(p.theta, -1.5f);
    EXPECT_GT(p.theta, -3.0f);
}

TEST(SimWorldTest, DrivesInCurveWhenWheelsDiffer)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({0.0f, 50.0f, 0.0f});
    sim.setMotorCommand(0, 50);   // left slower
    sim.setMotorCommand(1, 100);  // right faster → curves left (CCW)
    run(sim, 1.0f);

    const auto p = sim.pose();
    EXPECT_GT(p.x, 0.0f);
    EXPECT_GT(p.theta, 0.0f);     // rotated CCW
    EXPECT_GT(p.y, 50.0f);        // curved "up" in +Y
}

TEST(SimWorldTest, InversionFlipsWheelDirection)
{
    auto motors = defaultMotors();
    motors.leftInverted = true;
    motors.rightInverted = true;

    SimWorld sim;
    sim.configure(defaultRobot(), motors);
    sim.setPose({100.0f, 50.0f, 0.0f});
    sim.setMotorCommand(0, 100);
    sim.setMotorCommand(1, 100);
    run(sim, 1.0f);

    // Both inverted → both wheels spin backward → robot drives −X.
    EXPECT_LT(sim.pose().x, 100.0f);
}

TEST(SimWorldTest, MotorRampUpIsMonotonic)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({0.0f, 0.0f, 0.0f});
    sim.setMotorCommand(0, 100);
    sim.setMotorCommand(1, 100);

    float lastOmega = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        sim.tick(0.01f);
        const float omega = sim.wheelOmegaLeft();
        EXPECT_GE(omega, lastOmega);
        lastOmega = omega;
    }
    // After 200 ms with τ=50 ms, should be ≈ 98% of max (1 − e^-4).
    EXPECT_GT(lastOmega, 0.95f * 30.0f);
}

TEST(SimWorldTest, WallStopsForwardMotion)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());

    WorldMap map;
    map.setTable(200.0f, 100.0f);
    sim.setMap(std::move(map));

    sim.setPose({20.0f, 50.0f, 0.0f});
    sim.setMotorCommand(0, 100);
    sim.setMotorCommand(1, 100);
    run(sim, 5.0f);  // long enough to traverse the table

    const auto p = sim.pose();
    // Right border at x=200, half length = 9 → should stop near x ≈ 191.
    EXPECT_LE(p.x, 191.0f + 0.5f);
    EXPECT_GT(p.x, 170.0f);
}

TEST(SimWorldTest, SensorMountWorldPositionUnrotated)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({10.0f, 20.0f, 0.0f});
    SensorMount mount{"front_left", 5.0f, 2.0f, 0, 0.0f};
    const Vec2 world = sim.sensorWorldPosition(mount);
    EXPECT_NEAR(world.x, 15.0f, 1e-4f);
    EXPECT_NEAR(world.y, 22.0f, 1e-4f);
}

TEST(SimWorldTest, SensorMountWorldPositionRotated90)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());
    sim.setPose({10.0f, 20.0f, kPi / 2.0f});
    // Mount is 5cm forward, 2cm left in robot frame.
    // At 90° heading: forward = +Y, left = −X.
    // So world offset: (-2, +5) → world = (10-2, 20+5) = (8, 25).
    SensorMount mount{"front_left", 5.0f, 2.0f, 0, 0.0f};
    const Vec2 world = sim.sensorWorldPosition(mount);
    EXPECT_NEAR(world.x, 8.0f, 1e-4f);
    EXPECT_NEAR(world.y, 25.0f, 1e-4f);
}

TEST(SimWorldTest, LineSensorDetectsLineUnderneath)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());

    WorldMap map;
    map.setTable(200.0f, 100.0f);
    // Vertical black line at x=100, spanning y=0..100.
    map.addSegment({MapSegment::Kind::Line, 100.0f, 0.0f, 100.0f, 100.0f, 1.0f});
    sim.setMap(std::move(map));

    // Robot at x=99, sensor 1cm forward in robot frame, heading +X.
    sim.setPose({99.0f, 50.0f, 0.0f});
    SensorMount mount{"front", 1.0f, 0.0f, 0, 0.0f};
    EXPECT_TRUE(sim.isSensorOnBlackLine(mount));

    // Move robot away — sensor should no longer detect.
    sim.setPose({80.0f, 50.0f, 0.0f});
    EXPECT_FALSE(sim.isSensorOnBlackLine(mount));
}

TEST(SimWorldTest, DriveForwardWithLineSensorCrossingStripe)
{
    SimWorld sim;
    sim.configure(defaultRobot(), defaultMotors());

    WorldMap map;
    map.setTable(300.0f, 100.0f);
    map.addSegment({MapSegment::Kind::Line, 150.0f, 0.0f, 150.0f, 100.0f, 2.0f});
    sim.setMap(std::move(map));

    sim.setPose({50.0f, 50.0f, 0.0f});
    SensorMount mount{"bottom", 0.0f, 0.0f, 0, 0.0f};

    sim.setMotorCommand(0, 100);
    sim.setMotorCommand(1, 100);

    bool sawBlack = false;
    for (int i = 0; i < 300; ++i)
    {
        sim.tick(0.01f);
        if (sim.isSensorOnBlackLine(mount)) sawBlack = true;
    }
    EXPECT_TRUE(sawBlack);
    // Should have crossed the line and kept going.
    EXPECT_GT(sim.pose().x, 160.0f);
}

TEST(SimWorldTest, DeterministicUnderFixedStep)
{
    // Two independent SimWorld instances with identical inputs and dt should
    // produce bit-identical poses.
    const auto runOne = [] {
        SimWorld sim;
        sim.configure(defaultRobot(), defaultMotors());
        sim.setPose({10.0f, 20.0f, 0.3f});
        sim.setMotorCommand(0, 75);
        sim.setMotorCommand(1, 50);
        run(sim, 1.5f);
        return sim.pose();
    };
    const auto a = runOne();
    const auto b = runOne();
    EXPECT_FLOAT_EQ(a.x, b.x);
    EXPECT_FLOAT_EQ(a.y, b.y);
    EXPECT_FLOAT_EQ(a.theta, b.theta);
}
