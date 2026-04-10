#include <gtest/gtest.h>

#include "libstp/sim/Collision.hpp"
#include "libstp/sim/WorldMap.hpp"

#include <cmath>

using namespace libstp::sim;
using namespace libstp::sim::collision;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTol = 0.02f;

    RobotConfig makeRobot(float widthCm = 18.0f, float lengthCm = 18.0f)
    {
        RobotConfig r{};
        r.widthCm = widthCm;
        r.lengthCm = lengthCm;
        r.rotationCenterForwardCm = 0.0f;
        r.rotationCenterStrafeCm = 0.0f;
        return r;
    }

    WorldMap makeOpenTable(float w = 400.0f, float h = 300.0f)
    {
        WorldMap map;
        map.setTable(w, h);
        return map;
    }
}

TEST(CollisionTest, NoWallsNoOp)
{
    const auto robot = makeRobot();
    const std::vector<MapSegment> walls;
    const Pose2D start{50.0f, 50.0f, 0.0f};
    const Pose2D target{80.0f, 50.0f, 0.0f};
    const auto path = simulateSegment(start, target, robot, walls);
    ASSERT_FALSE(path.empty());
    const auto& last = path.back();
    EXPECT_NEAR(last.x, 80.0f, kTol);
    EXPECT_NEAR(last.y, 50.0f, kTol);
}

TEST(CollisionTest, StopsAtTableBorder)
{
    const auto robot = makeRobot(18.0f, 18.0f);
    auto map = makeOpenTable(100.0f, 100.0f);
    const auto walls = buildCollisionWalls(map);

    // Start at (20, 50), drive far east past the right border.
    // Robot half-length = 9, so forward face hits wall at x=100 when center x = 91.
    const Pose2D start{20.0f, 50.0f, 0.0f};
    const Pose2D target{200.0f, 50.0f, 0.0f};
    const auto path = simulateSegment(start, target, robot, walls);

    ASSERT_FALSE(path.empty());
    const auto& last = path.back();
    EXPECT_LE(last.x, 91.0f + kTol);
    EXPECT_GT(last.x, 80.0f);
    EXPECT_NEAR(last.y, 50.0f, kTol);
}

TEST(CollisionTest, SlidesAlongWall)
{
    // Robot heading 45° into a table border — should slide north once it hits
    // the east wall, ending up at the NE corner region.
    const auto robot = makeRobot(10.0f, 10.0f);
    auto map = makeOpenTable(100.0f, 100.0f);
    const auto walls = buildCollisionWalls(map);

    const Pose2D start{50.0f, 50.0f, kPi / 4.0f};
    const Pose2D target{200.0f, 200.0f, kPi / 4.0f};
    const auto path = simulateSegment(start, target, robot, walls);

    ASSERT_FALSE(path.empty());
    const auto& last = path.back();
    // Sliding should bring us toward the NE corner region.
    EXPECT_GT(last.x, 80.0f);
    EXPECT_GT(last.y, 80.0f);
    EXPECT_LE(last.x, 95.0f + kTol);
    EXPECT_LE(last.y, 95.0f + kTol);
}

TEST(CollisionTest, DetectsExistingIntersection)
{
    const auto robot = makeRobot(20.0f, 20.0f);
    auto map = makeOpenTable(100.0f, 100.0f);
    const auto walls = buildCollisionWalls(map);

    // Robot center at (5, 50). Half-length = 10, so left face is at x = -5
    // which pokes through the west wall.
    const Pose2D inside{50.0f, 50.0f, 0.0f};
    const Pose2D poking{5.0f, 50.0f, 0.0f};
    EXPECT_FALSE(checkRobotCollision(inside, robot, walls));
    EXPECT_TRUE(checkRobotCollision(poking, robot, walls));
}

TEST(CollisionTest, FreeSpaceCollisionIsNull)
{
    const auto robot = makeRobot();
    auto map = makeOpenTable(400.0f, 300.0f);
    const auto walls = buildCollisionWalls(map);
    const Pose2D pose{200.0f, 150.0f, 0.0f};
    const Vec2 moveVec{1.0f, 0.0f};
    EXPECT_FALSE(findCollision(pose, moveVec, robot, walls).has_value());
}

TEST(CollisionTest, InternalWallBlocksMotion)
{
    const auto robot = makeRobot(10.0f, 10.0f);
    WorldMap map;
    map.setTable(200.0f, 200.0f);
    // A horizontal wall at y=100 from x=0 to x=200.
    map.addSegment({MapSegment::Kind::Wall, 0.0f, 100.0f, 200.0f, 100.0f, 0.0f});
    const auto walls = buildCollisionWalls(map);

    // Robot starts at y=50, drives straight up toward y=180. The wall should
    // stop it near y=95 (wall y=100 minus half-length 5).
    const Pose2D start{100.0f, 50.0f, kPi / 2.0f};
    const Pose2D target{100.0f, 180.0f, kPi / 2.0f};
    const auto path = simulateSegment(start, target, robot, walls);

    ASSERT_FALSE(path.empty());
    const auto& last = path.back();
    EXPECT_LE(last.y, 95.0f + kTol);
    EXPECT_GT(last.y, 80.0f);
}

TEST(CollisionTest, BuildCollisionWallsExpandsThickWall)
{
    WorldMap map;
    map.setTable(100.0f, 100.0f);
    // A 2cm-thick wall — should expand to 4 edges (plus 4 table borders).
    map.addSegment({MapSegment::Kind::Wall, 20.0f, 50.0f, 80.0f, 50.0f, 2.0f});
    const auto walls = buildCollisionWalls(map);
    EXPECT_EQ(walls.size(), 8u);
}
