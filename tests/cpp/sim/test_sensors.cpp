#include <gtest/gtest.h>

#include "libstp/sim/Collision.hpp"
#include "libstp/sim/SimWorld.hpp"

#include <cmath>

using namespace libstp::sim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    RobotConfig tinyRobot()
    {
        RobotConfig r{};
        r.widthCm = 10.0f;
        r.lengthCm = 10.0f;
        r.wheelRadiusM = 0.03f;
        r.trackWidthM = 0.1f;
        return r;
    }

    SimMotorMap defaultMotors()
    {
        return SimMotorMap{};
    }
}

// ─── Raycast ───

TEST(RaycastTest, HitsWallStraightAhead)
{
    std::vector<MapSegment> walls;
    walls.push_back({MapSegment::Kind::Wall, 50.0f, -50.0f, 50.0f, 50.0f, 0.0f});
    const float d = collision::raycastDistanceCm(0.0f, 0.0f, 0.0f, 200.0f, walls);
    EXPECT_NEAR(d, 50.0f, 1e-3f);
}

TEST(RaycastTest, MissReturnsMaxRange)
{
    std::vector<MapSegment> walls;
    walls.push_back({MapSegment::Kind::Wall, 50.0f, -50.0f, 50.0f, 50.0f, 0.0f});
    // Aim opposite direction — ray doesn't reach the wall.
    const float d = collision::raycastDistanceCm(0.0f, 0.0f, kPi, 30.0f, walls);
    EXPECT_FLOAT_EQ(d, 30.0f);
}

TEST(RaycastTest, PicksNearestOfMultipleWalls)
{
    std::vector<MapSegment> walls;
    walls.push_back({MapSegment::Kind::Wall, 80.0f, -50.0f, 80.0f, 50.0f, 0.0f});
    walls.push_back({MapSegment::Kind::Wall, 40.0f, -50.0f, 40.0f, 50.0f, 0.0f});
    const float d = collision::raycastDistanceCm(0.0f, 0.0f, 0.0f, 200.0f, walls);
    EXPECT_NEAR(d, 40.0f, 1e-3f);
}

TEST(RaycastTest, ParallelWallIsMissed)
{
    std::vector<MapSegment> walls;
    walls.push_back({MapSegment::Kind::Wall, 0.0f, 10.0f, 100.0f, 10.0f, 0.0f});
    const float d = collision::raycastDistanceCm(0.0f, 0.0f, 0.0f, 200.0f, walls);
    EXPECT_FLOAT_EQ(d, 200.0f);
}

TEST(RaycastTest, AngledRayHitsCorrectSegment)
{
    std::vector<MapSegment> walls;
    walls.push_back({MapSegment::Kind::Wall, 0.0f, 10.0f, 100.0f, 10.0f, 0.0f});
    // 45° ray from origin should hit y=10 at x=10 → distance = sqrt(200) ≈ 14.142.
    const float d = collision::raycastDistanceCm(0.0f, 0.0f, kPi / 4.0f, 200.0f, walls);
    EXPECT_NEAR(d, std::sqrt(200.0f), 1e-3f);
}

// ─── SimWorld line sensor sampling ───

TEST(SimSensorTest, LineSensorReturnsZeroOnBlackAnd1023OnWhite)
{
    SimWorld sim;
    sim.configure(tinyRobot(), defaultMotors());

    WorldMap map;
    map.setTable(200.0f, 100.0f);
    map.addSegment({MapSegment::Kind::Line, 100.0f, 0.0f, 100.0f, 100.0f, 2.0f});
    sim.setMap(std::move(map));

    sim.attachLineSensor(2, 0.0f, 0.0f, "center");

    sim.setPose({100.0f, 50.0f, 0.0f});
    auto v = sim.readAnalog(2);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 0);

    sim.setPose({50.0f, 50.0f, 0.0f});
    v = sim.readAnalog(2);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 1023);
}

TEST(SimSensorTest, UnregisteredPortReturnsNullopt)
{
    SimWorld sim;
    sim.configure(tinyRobot(), defaultMotors());
    EXPECT_FALSE(sim.readAnalog(0).has_value());
}

TEST(SimSensorTest, DetachedSensorNoLongerSamples)
{
    SimWorld sim;
    sim.configure(tinyRobot(), defaultMotors());
    WorldMap map;
    map.setTable(100.0f, 100.0f);
    sim.setMap(std::move(map));

    sim.attachLineSensor(2, 0.0f, 0.0f);
    sim.setPose({50.0f, 50.0f, 0.0f});
    EXPECT_TRUE(sim.readAnalog(2).has_value());

    sim.detachSensor(2);
    EXPECT_FALSE(sim.readAnalog(2).has_value());
}

// ─── SimWorld distance sensor (ET polynomial) ───

TEST(SimSensorTest, DistanceSensorFarFromWall)
{
    SimWorld sim;
    sim.configure(tinyRobot(), defaultMotors());
    WorldMap map;
    map.setTable(500.0f, 500.0f);
    sim.setMap(std::move(map));

    sim.attachDistanceSensor(3, 0.0f, 0.0f, 0.0f, 100.0f);
    sim.setPose({250.0f, 250.0f, 0.0f});
    // Nearest wall is the table border at x=500 → distance ≈ 250 cm, out of
    // the 100 cm max range → sensor returns the "no target" floor (275).
    const auto v = sim.readAnalog(3);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 275);
}

TEST(SimSensorTest, DistanceSensorCloseToWallReadsHigh)
{
    SimWorld sim;
    sim.configure(tinyRobot(), defaultMotors());
    WorldMap map;
    map.setTable(200.0f, 100.0f);
    sim.setMap(std::move(map));

    sim.attachDistanceSensor(3, 0.0f, 0.0f, 0.0f, 100.0f);
    // Aim the sensor at the east border at x=200. Place robot 15 cm away.
    sim.setPose({185.0f, 50.0f, 0.0f});
    auto v = sim.readAnalog(3);
    ASSERT_TRUE(v.has_value());
    // 15 cm falls in the useful range (10-45 cm) of the KIPR ET polynomial.
    // Expected value: 307.75 − 279.3·x + 180.9·x² where x = -1.2222 + 0.0444·15.
    const float x15 = -1.2222f + 0.0444f * 15.0f;
    const float expected = 307.75f - 279.3f * x15 + 180.9f * (x15 * x15);
    EXPECT_NEAR(*v, static_cast<uint16_t>(expected), 2);

    // Move closer to 30 cm → value should be lower (ET is monotonic in range).
    sim.setPose({170.0f, 50.0f, 0.0f});
    const uint16_t v30 = *sim.readAnalog(3);
    const float x30 = -1.2222f + 0.0444f * 30.0f;
    const float expected30 = 307.75f - 279.3f * x30 + 180.9f * (x30 * x30);
    EXPECT_NEAR(v30, static_cast<uint16_t>(expected30), 2);
}

TEST(SimSensorTest, DistanceSensorMountAngleAimsSideways)
{
    SimWorld sim;
    sim.configure(tinyRobot(), defaultMotors());
    WorldMap map;
    map.setTable(200.0f, 100.0f);
    sim.setMap(std::move(map));

    // Sensor mounted facing +Y (robot-local left, angle = π/2) from robot center.
    sim.attachDistanceSensor(3, 0.0f, 0.0f, kPi / 2.0f, 100.0f);
    // Place robot in the middle, heading +X. Sensor aims +Y → hits top border
    // at y=100. Robot is at y=70, distance = 30 cm.
    sim.setPose({100.0f, 70.0f, 0.0f});
    const uint16_t v = *sim.readAnalog(3);

    // Independently compute the expected value from the 30 cm formula.
    const float x = -1.2222f + 0.0444f * 30.0f;
    const float expected = 307.75f - 279.3f * x + 180.9f * (x * x);
    EXPECT_NEAR(v, static_cast<uint16_t>(expected), 2);
}
