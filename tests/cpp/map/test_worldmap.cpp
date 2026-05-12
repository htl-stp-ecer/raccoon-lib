#include <gtest/gtest.h>

#include "libstp/map/Geometry.hpp"
#include "libstp/map/MapSegment.hpp"
#include "libstp/map/WorldMap.hpp"

#include <cmath>
#include <numbers>

using namespace libstp::map;

namespace
{
    constexpr float kPi = std::numbers::pi_v<float>;

    constexpr const char* kSingleLineFtmap = R"({
  "format": "flowchart-table-map",
  "version": 1,
  "table": { "widthCm": 200, "heightCm": 100 },
  "lines": [
    { "kind": "line", "startX": 50, "startY": 50, "endX": 150, "endY": 50, "widthCm": 1.5 }
  ]
})";

    constexpr const char* kWallBoxFtmap = R"({
  "format": "flowchart-table-map",
  "version": 1,
  "table": { "widthCm": 100, "heightCm": 100 },
  "lines": [
    { "kind": "wall", "startX": 40, "startY": 40, "endX": 60, "endY": 40, "widthCm": 0 },
    { "kind": "wall", "startX": 60, "startY": 40, "endX": 60, "endY": 60, "widthCm": 0 }
  ]
})";
}

TEST(MapWorldMapTest, ParsesTableDimensions)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);
    EXPECT_FLOAT_EQ(map.tableWidthCm(), 200.0f);
    EXPECT_FLOAT_EQ(map.tableHeightCm(), 100.0f);
}

TEST(MapWorldMapTest, ParsesLineSegmentsWithYFlip)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);

    const auto lines = map.lines();
    ASSERT_EQ(lines.size(), 1u);

    // ftmap Y=50 in a 100-tall table → bottom-up Y=50 (symmetric case).
    EXPECT_FLOAT_EQ(lines[0].startY, 50.0f);
    EXPECT_FLOAT_EQ(lines[0].endY, 50.0f);
}

TEST(MapWorldMapTest, IsOnLineHitsAndMisses)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);

    EXPECT_TRUE(map.isOnLine(100.0f, 50.0f));
    EXPECT_TRUE(map.isOnBlackLine(100.0f, 50.0f));   // alias matches
    EXPECT_FALSE(map.isOnLine(100.0f, 60.0f));
    EXPECT_FALSE(map.isOnLine(10.0f, 50.0f));
}

TEST(MapWorldMapTest, IsOnWallIncludesBorderEdges)
{
    WorldMap map;
    map.parseFtmap(kWallBoxFtmap);
    // Inside the explicit walls (Y is flipped on ingest: ftmap y=40 in a
    // 100-tall table → world y=60).
    EXPECT_TRUE(map.isOnWall(50.0f, 60.0f));
    EXPECT_TRUE(map.isOnWall(60.0f, 50.0f));
    // Border edges are 100x100 — corner / edge points must register
    EXPECT_TRUE(map.isOnWall(0.0f, 50.0f));
    EXPECT_TRUE(map.isOnWall(100.0f, 50.0f));
    EXPECT_TRUE(map.isOnWall(50.0f, 0.0f));
    EXPECT_TRUE(map.isOnWall(50.0f, 100.0f));
    // Interior away from walls
    EXPECT_FALSE(map.isOnWall(20.0f, 80.0f));
}

TEST(MapWorldMapTest, WallsListIncludesFourBorders)
{
    WorldMap map;
    map.parseFtmap(kWallBoxFtmap);
    const auto walls = map.walls();
    EXPECT_EQ(walls.size(), 6u);
}

TEST(MapWorldMapTest, DistanceToNearestLine)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);
    // Line is at y=50, x from 50..150. Point at (100, 60) is 10 cm above
    // the centerline.
    EXPECT_NEAR(map.distanceToNearestLine(100.0f, 60.0f), 10.0f, 1e-4f);
    // Off the end of the segment — distance from endpoint.
    EXPECT_NEAR(map.distanceToNearestLine(150.0f, 53.0f), 3.0f, 1e-4f);
}

TEST(MapWorldMapTest, DistanceToNearestLineEmptyMapIsInfinite)
{
    WorldMap map;
    map.setTable(100.0f, 100.0f);
    EXPECT_TRUE(std::isinf(map.distanceToNearestLine(50.0f, 50.0f)));
}

TEST(MapWorldMapTest, DistanceToNearestWallIncludesBorders)
{
    WorldMap map;
    map.setTable(100.0f, 100.0f);
    // No explicit walls — distance comes from the nearest border.
    EXPECT_NEAR(map.distanceToNearestWall(10.0f, 50.0f), 10.0f, 1e-4f);
    EXPECT_NEAR(map.distanceToNearestWall(50.0f, 90.0f), 10.0f, 1e-4f);
}

TEST(MapWorldMapTest, SensorFieldPositionForwardAlongHeading)
{
    WorldMap map;
    map.setTable(200.0f, 200.0f);
    SensorOffset s{10.0f, 0.0f};   // 10 cm forward, 0 strafe

    // Heading 0 = +X
    auto p = map.sensorFieldPosition(50.0f, 50.0f, 0.0f, s);
    EXPECT_NEAR(p.xCm, 60.0f, 1e-4f);
    EXPECT_NEAR(p.yCm, 50.0f, 1e-4f);

    // Heading 90° = +Y
    p = map.sensorFieldPosition(50.0f, 50.0f, kPi / 2.0f, s);
    EXPECT_NEAR(p.xCm, 50.0f, 1e-4f);
    EXPECT_NEAR(p.yCm, 60.0f, 1e-4f);

    // Heading 180° = -X
    p = map.sensorFieldPosition(50.0f, 50.0f, kPi, s);
    EXPECT_NEAR(p.xCm, 40.0f, 1e-4f);
    EXPECT_NEAR(p.yCm, 50.0f, 1e-4f);
}

TEST(MapWorldMapTest, SensorFieldPositionStrafeIs90CCW)
{
    WorldMap map;
    map.setTable(200.0f, 200.0f);
    // Strafe 5 cm with no forward — at heading 0 this should land on +Y
    // (90° CCW of heading is the robot's left).
    SensorOffset s{0.0f, 5.0f};
    auto p = map.sensorFieldPosition(50.0f, 50.0f, 0.0f, s);
    EXPECT_NEAR(p.xCm, 50.0f, 1e-4f);
    EXPECT_NEAR(p.yCm, 55.0f, 1e-4f);
}

TEST(MapWorldMapTest, SensorIsOnLineRespectsRotation)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);  // line y=50, 50<=x<=150
    SensorOffset s{10.0f, 0.0f};

    // Robot at (90, 50) heading 0 → sensor at (100, 50) — on the line.
    EXPECT_TRUE(map.sensorIsOnLine(90.0f, 50.0f, 0.0f, s));
    // Same robot but heading 90° → sensor at (90, 60) — off the line.
    EXPECT_FALSE(map.sensorIsOnLine(90.0f, 50.0f, kPi / 2.0f, s));
    // Heading 180° → sensor at (80, 50) — back on the line.
    EXPECT_TRUE(map.sensorIsOnLine(90.0f, 50.0f, kPi, s));
}

TEST(MapWorldMapTest, RejectsBadFormat)
{
    WorldMap map;
    EXPECT_THROW(map.parseFtmap(R"({"format":"wrong","version":1,"table":{"widthCm":1,"heightCm":1}})"),
                 FtmapParseError);
}

TEST(MapWorldMapTest, MapSegmentLengthProperty)
{
    MapSegment seg{MapSegment::Kind::Line, 0.0f, 0.0f, 3.0f, 4.0f, 1.0f};
    EXPECT_FLOAT_EQ(seg.length(), 5.0f);
}
