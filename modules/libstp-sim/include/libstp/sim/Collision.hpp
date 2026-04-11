#pragma once

#include "libstp/sim/Pose2D.hpp"
#include "libstp/sim/RobotConfig.hpp"
#include "libstp/sim/WorldMap.hpp"

#include <optional>
#include <vector>

namespace libstp::sim::collision
{
    /// Expand thick walls into 4-edge rectangles, append table borders.
    /// Port of web-ide physics.ts buildCollisionWalls().
    std::vector<MapSegment> buildCollisionWalls(const WorldMap& map);

    struct CollisionInfo
    {
        MapSegment wall{};
        Vec2 normal{};
        Vec2 tangent{};
        float depth{0.0f};
    };

    /// Deepest wall the robot's OBB currently penetrates, if any.
    /// Port of physics.ts findCollision(). `moveVec` is the motion that was
    /// attempted; walls facing away from it are ignored.
    std::optional<CollisionInfo> findCollision(
        const Pose2D& pose,
        const Vec2& moveVec,
        const RobotConfig& robot,
        const std::vector<MapSegment>& walls);

    /// Push a penetrating pose out along the collision normal.
    /// Port of physics.ts resolvePoseAgainstWall().
    Pose2D resolveAgainstWall(
        const Pose2D& pose,
        const CollisionInfo& collision,
        const RobotConfig& robot);

    /// Integrate a motion from start → target with per-step wall collision and
    /// tangent sliding. Port of physics.ts simulateSegment().
    /// Returns the trail of poses the robot actually visited (empty if no motion).
    std::vector<Pose2D> simulateSegment(
        const Pose2D& start,
        const Pose2D& target,
        const RobotConfig& robot,
        const std::vector<MapSegment>& walls);

    /// True if the robot's OBB at `pose` intersects any wall.
    /// Port of physics.ts checkRobotCollision().
    bool checkRobotCollision(
        const Pose2D& pose,
        const RobotConfig& robot,
        const std::vector<MapSegment>& walls);

    /// Closest-hit raycast against a set of wall segments.
    /// `originX, originY` in cm, `angleRad` is the world-frame direction.
    /// Returns the distance (cm) to the first wall hit within `maxDistanceCm`,
    /// or `maxDistanceCm` if no wall is hit (clean miss to max range).
    float raycastDistanceCm(
        float originX,
        float originY,
        float angleRad,
        float maxDistanceCm,
        const std::vector<MapSegment>& walls);
}
