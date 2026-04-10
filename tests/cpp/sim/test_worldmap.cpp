#include <gtest/gtest.h>

#include "libstp/sim/WorldMap.hpp"

using namespace libstp::sim;

namespace
{
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

    constexpr const char* kMixedFtmap = R"({
  "format": "flowchart-table-map",
  "version": 1,
  "table": { "widthCm": 150, "heightCm": 80 },
  "lines": [
    { "kind": "line", "startX": 10, "startY": 10, "endX": 100, "endY": 10, "widthCm": 2 },
    { "kind": "wall", "startX": 0, "startY": 40, "endX": 150, "endY": 40, "widthCm": 0 }
  ]
})";
}

TEST(WorldMapTest, ParsesTableDimensions)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);
    EXPECT_FLOAT_EQ(map.tableWidthCm(), 200.0f);
    EXPECT_FLOAT_EQ(map.tableHeightCm(), 100.0f);
}

TEST(WorldMapTest, ParsesLineSegments)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);

    const auto lines = map.lines();
    ASSERT_EQ(lines.size(), 1u);

    // ftmap Y is top-origin, worldmap Y is bottom-origin, so Y is flipped:
    // ftmap Y=50 in a 100-tall table → worldmap Y=50.
    EXPECT_FLOAT_EQ(lines[0].startX, 50.0f);
    EXPECT_FLOAT_EQ(lines[0].startY, 50.0f);
    EXPECT_FLOAT_EQ(lines[0].endX, 150.0f);
    EXPECT_FLOAT_EQ(lines[0].endY, 50.0f);
    EXPECT_FLOAT_EQ(lines[0].widthCm, 1.5f);
}

TEST(WorldMapTest, YIsFlippedFromFtmap)
{
    // A wall at ftmap Y=10 in a 100-tall table must land at worldmap Y=90.
    constexpr const char* kYFlip = R"({
      "format": "flowchart-table-map",
      "version": 1,
      "table": { "widthCm": 100, "heightCm": 100 },
      "lines": [
        { "kind": "wall", "startX": 0, "startY": 10, "endX": 100, "endY": 10, "widthCm": 0 }
      ]
    })";

    WorldMap map;
    map.parseFtmap(kYFlip);
    const auto walls = map.segments();
    ASSERT_EQ(walls.size(), 1u);
    EXPECT_FLOAT_EQ(walls[0].startY, 90.0f);
    EXPECT_FLOAT_EQ(walls[0].endY, 90.0f);
}

TEST(WorldMapTest, IsOnBlackLineCenterHit)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);
    EXPECT_TRUE(map.isOnBlackLine(100.0f, 50.0f));
    EXPECT_TRUE(map.isOnBlackLine(50.1f, 50.0f));
    EXPECT_TRUE(map.isOnBlackLine(149.9f, 50.0f));
}

TEST(WorldMapTest, IsOnBlackLineMissesWhenFar)
{
    WorldMap map;
    map.parseFtmap(kSingleLineFtmap);
    EXPECT_FALSE(map.isOnBlackLine(100.0f, 60.0f));
    EXPECT_FALSE(map.isOnBlackLine(10.0f, 50.0f));
    EXPECT_FALSE(map.isOnBlackLine(190.0f, 50.0f));
}

TEST(WorldMapTest, IsOnBlackLineRespectsMinThreshold)
{
    // A very thin line still has a 0.75cm detection threshold.
    constexpr const char* kThin = R"({
      "format": "flowchart-table-map",
      "version": 1,
      "table": { "widthCm": 100, "heightCm": 100 },
      "lines": [
        { "kind": "line", "startX": 50, "startY": 50, "endX": 60, "endY": 50, "widthCm": 0.1 }
      ]
    })";
    WorldMap map;
    map.parseFtmap(kThin);
    EXPECT_TRUE(map.isOnBlackLine(55.0f, 50.7f));
    EXPECT_FALSE(map.isOnBlackLine(55.0f, 51.0f));
}

TEST(WorldMapTest, WallsIncludeTableBorders)
{
    WorldMap map;
    map.parseFtmap(kWallBoxFtmap);
    const auto walls = map.walls();
    // 2 explicit + 4 borders
    EXPECT_EQ(walls.size(), 6u);
}

TEST(WorldMapTest, MixedScene)
{
    WorldMap map;
    map.parseFtmap(kMixedFtmap);
    EXPECT_EQ(map.lines().size(), 1u);
    // 1 explicit wall + 4 borders
    EXPECT_EQ(map.walls().size(), 5u);
    EXPECT_EQ(map.segments().size(), 2u);
}

TEST(WorldMapTest, RejectsBadFormat)
{
    constexpr const char* kBad = R"({
      "format": "wrong",
      "version": 1,
      "table": { "widthCm": 100, "heightCm": 100 },
      "lines": []
    })";
    WorldMap map;
    EXPECT_THROW(map.parseFtmap(kBad), FtmapParseError);
}

TEST(WorldMapTest, RejectsBadVersion)
{
    constexpr const char* kBad = R"({
      "format": "flowchart-table-map",
      "version": 2,
      "table": { "widthCm": 100, "heightCm": 100 },
      "lines": []
    })";
    WorldMap map;
    EXPECT_THROW(map.parseFtmap(kBad), FtmapParseError);
}

TEST(WorldMapTest, RejectsBadKind)
{
    constexpr const char* kBad = R"({
      "format": "flowchart-table-map",
      "version": 1,
      "table": { "widthCm": 100, "heightCm": 100 },
      "lines": [
        { "kind": "banana", "startX": 0, "startY": 0, "endX": 10, "endY": 0, "widthCm": 1 }
      ]
    })";
    WorldMap map;
    EXPECT_THROW(map.parseFtmap(kBad), FtmapParseError);
}
