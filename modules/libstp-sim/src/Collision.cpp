#include "libstp/sim/Collision.hpp"

#include <algorithm>
#include <cmath>

namespace libstp::sim::collision
{
    namespace
    {
        constexpr float kEps = 1e-6f;
        constexpr float kStepCm = 1.0f;
        constexpr float kStepAngleRad = 3.14159265358979323846f / 36.0f;
        constexpr int kMaxIterations = 4000;
        constexpr float kMinWallThicknessCm = 0.01f;

        float dot(const Vec2& a, const Vec2& b) noexcept { return a.x * b.x + a.y * b.y; }
        float length(const Vec2& v) noexcept { return std::sqrt(v.x * v.x + v.y * v.y); }
        Vec2 scale(const Vec2& v, float s) noexcept { return {v.x * s, v.y * s}; }
        Vec2 negate(const Vec2& v) noexcept { return {-v.x, -v.y}; }

        struct Rectangle
        {
            Vec2 center;
            Vec2 forward;
            Vec2 left;
            float halfLength{0.0f};
            float halfWidth{0.0f};
        };

        /// Port of physics.ts getRectangle(). Builds the robot's oriented
        /// bounding box in world coordinates, with the rotation center offset
        /// applied in the robot's local frame.
        Rectangle getRectangle(const Pose2D& pose, const RobotConfig& robot)
        {
            const float c = std::cos(pose.theta);
            const float s = std::sin(pose.theta);
            const Vec2 forward{c, s};
            const Vec2 left{-s, c};

            const float centerLocalX = -robot.rotationCenterForwardCm;
            const float centerLocalY = robot.rotationCenterStrafeCm;
            const Vec2 center{
                pose.x + centerLocalX * c - centerLocalY * s,
                pose.y + centerLocalX * s + centerLocalY * c,
            };

            return {
                center,
                forward,
                left,
                robot.lengthCm * 0.5f,
                robot.widthCm * 0.5f,
            };
        }

        /// Expand one thick wall into 4 thin edges. Port of physics.ts expandWallSegment().
        std::vector<MapSegment> expandWallSegment(const MapSegment& seg, float thicknessCm)
        {
            const float dx = seg.endX - seg.startX;
            const float dy = seg.endY - seg.startY;
            const float len = std::hypot(dx, dy);
            if (len <= kEps) return {};

            const float halfThickness = thicknessCm * 0.5f;
            const float nx = -dy / len;
            const float ny = dx / len;
            const float ox = nx * halfThickness;
            const float oy = ny * halfThickness;

            const Vec2 a{seg.startX + ox, seg.startY + oy};
            const Vec2 b{seg.endX + ox, seg.endY + oy};
            const Vec2 cc{seg.endX - ox, seg.endY - oy};
            const Vec2 d{seg.startX - ox, seg.startY - oy};

            return {
                {MapSegment::Kind::Wall, a.x, a.y, b.x, b.y, 0.0f},
                {MapSegment::Kind::Wall, b.x, b.y, cc.x, cc.y, 0.0f},
                {MapSegment::Kind::Wall, cc.x, cc.y, d.x, d.y, 0.0f},
                {MapSegment::Kind::Wall, d.x, d.y, a.x, a.y, 0.0f},
            };
        }

        /// Port of physics.ts computeCollision(). OBB-vs-segment SAT returning
        /// penetration depth along the wall normal, or nullopt if clear.
        std::optional<CollisionInfo> computeCollision(
            const Pose2D& pose,
            const RobotConfig& robot,
            const MapSegment& wall)
        {
            const Vec2 wallVec{wall.endX - wall.startX, wall.endY - wall.startY};
            const float wallLength = length(wallVec);
            if (wallLength <= kEps) return std::nullopt;

            const Vec2 tangent = scale(wallVec, 1.0f / wallLength);
            const Vec2 normal{-tangent.y, tangent.x};
            const Rectangle rect = getRectangle(pose, robot);
            const Vec2 centerToStart{rect.center.x - wall.startX, rect.center.y - wall.startY};

            const float radiusAlongTangent =
                rect.halfLength * std::abs(dot(tangent, rect.forward)) +
                rect.halfWidth * std::abs(dot(tangent, rect.left));
            const float proj = dot(centerToStart, tangent);
            if (proj + radiusAlongTangent < 0.0f || proj - radiusAlongTangent > wallLength)
            {
                return std::nullopt;
            }

            const float support =
                rect.halfLength * std::abs(dot(normal, rect.forward)) +
                rect.halfWidth * std::abs(dot(normal, rect.left));
            const float signedDistance = dot(centerToStart, normal);
            const float absDistance = std::abs(signedDistance);

            if (absDistance < support - kEps)
            {
                const Vec2 correctionNormal = signedDistance >= 0.0f ? normal : negate(normal);
                return CollisionInfo{
                    wall,
                    correctionNormal,
                    tangent,
                    support - absDistance,
                };
            }
            return std::nullopt;
        }
    }

    std::vector<MapSegment> buildCollisionWalls(const WorldMap& map)
    {
        std::vector<MapSegment> out;
        for (const auto& seg : map.segments())
        {
            if (seg.kind != MapSegment::Kind::Wall) continue;
            const float thickness = std::max(0.0f, seg.widthCm);
            if (thickness < kMinWallThicknessCm)
            {
                out.push_back({MapSegment::Kind::Wall, seg.startX, seg.startY, seg.endX, seg.endY, 0.0f});
                continue;
            }
            auto expanded = expandWallSegment(seg, thickness);
            if (!expanded.empty())
            {
                out.insert(out.end(), expanded.begin(), expanded.end());
            }
            else
            {
                out.push_back({MapSegment::Kind::Wall, seg.startX, seg.startY, seg.endX, seg.endY, 0.0f});
            }
        }

        const float w = map.tableWidthCm();
        const float h = map.tableHeightCm();
        if (w > 0.0f && h > 0.0f)
        {
            out.push_back({MapSegment::Kind::Wall, 0.0f, 0.0f, w, 0.0f, 0.0f});
            out.push_back({MapSegment::Kind::Wall, w, 0.0f, w, h, 0.0f});
            out.push_back({MapSegment::Kind::Wall, w, h, 0.0f, h, 0.0f});
            out.push_back({MapSegment::Kind::Wall, 0.0f, h, 0.0f, 0.0f, 0.0f});
        }
        return out;
    }

    std::optional<CollisionInfo> findCollision(
        const Pose2D& pose,
        const Vec2& moveVec,
        const RobotConfig& robot,
        const std::vector<MapSegment>& walls)
    {
        std::optional<CollisionInfo> best;
        for (const auto& wall : walls)
        {
            auto hit = computeCollision(pose, robot, wall);
            if (!hit) continue;
            if (length(moveVec) > kEps && dot(moveVec, hit->normal) >= -kEps)
            {
                continue;
            }
            if (!best || hit->depth > best->depth)
            {
                best = hit;
            }
        }
        return best;
    }

    Pose2D resolveAgainstWall(
        const Pose2D& pose,
        const CollisionInfo& collision,
        const RobotConfig& robot)
    {
        const Rectangle rect = getRectangle(pose, robot);
        const Vec2 toStart{
            rect.center.x - collision.wall.startX,
            rect.center.y - collision.wall.startY,
        };
        const float support =
            rect.halfLength * std::abs(dot(collision.normal, rect.forward)) +
            rect.halfWidth * std::abs(dot(collision.normal, rect.left));
        const float distanceToWall = dot(toStart, collision.normal) - support;
        if (distanceToWall >= 0.0f) return pose;

        const float correction = -distanceToWall;
        return {
            pose.x + collision.normal.x * correction,
            pose.y + collision.normal.y * correction,
            pose.theta,
        };
    }

    std::vector<Pose2D> simulateSegment(
        const Pose2D& start,
        const Pose2D& target,
        const RobotConfig& robot,
        const std::vector<MapSegment>& walls)
    {
        std::vector<Pose2D> poses;
        Pose2D current = start;
        Vec2 remainingMove{target.x - current.x, target.y - current.y};
        float remainingTheta = normalizeAngle(target.theta - current.theta);

        int iterations = 0;
        while (iterations < kMaxIterations)
        {
            ++iterations;
            const float distance = length(remainingMove);
            const float angleAbs = std::abs(remainingTheta);

            if (distance <= kEps && angleAbs <= kEps)
            {
                break;
            }

            const float stepDist = distance > kEps ? std::min(kStepCm, distance) : 0.0f;
            const float stepTheta = angleAbs > kEps
                ? std::copysign(std::min(kStepAngleRad, angleAbs), remainingTheta)
                : 0.0f;
            const Vec2 direction = distance > kEps ? scale(remainingMove, 1.0f / distance) : Vec2{0.0f, 0.0f};
            const Vec2 stepMove = scale(direction, stepDist);
            const Pose2D attempted{
                current.x + stepMove.x,
                current.y + stepMove.y,
                normalizeAngle(current.theta + stepTheta),
            };

            auto collision = stepDist > kEps
                ? findCollision(attempted, stepMove, robot, walls)
                : std::nullopt;

            if (!collision)
            {
                current = attempted;
                remainingMove = {
                    remainingMove.x - stepMove.x,
                    remainingMove.y - stepMove.y,
                };
                remainingTheta = normalizeAngle(remainingTheta - stepTheta);
                if (stepDist > kEps || std::abs(stepTheta) > kEps)
                {
                    poses.push_back(current);
                }
                continue;
            }

            const Pose2D stopped = resolveAgainstWall(attempted, *collision, robot);
            const Vec2 movedVec{stopped.x - current.x, stopped.y - current.y};
            const Vec2 remainingAfter{
                remainingMove.x - movedVec.x,
                remainingMove.y - movedVec.y,
            };

            const Vec2 tangentDir = dot(remainingAfter, collision->tangent) >= 0.0f
                ? collision->tangent
                : negate(collision->tangent);
            const float remainingAlong = dot(remainingAfter, tangentDir);

            if (remainingAlong <= kEps)
            {
                const bool moved = std::abs(stopped.x - current.x) > kEps || std::abs(stopped.y - current.y) > kEps;
                const bool rotated = std::abs(normalizeAngle(stopped.theta - current.theta)) > kEps;
                if (moved || rotated)
                {
                    current = stopped;
                    poses.push_back(current);
                }
                break;
            }

            const Pose2D aligned{
                stopped.x,
                stopped.y,
                std::atan2(tangentDir.y, tangentDir.x),
            };
            const bool moved = std::abs(aligned.x - current.x) > kEps || std::abs(aligned.y - current.y) > kEps;
            const bool rotated = std::abs(normalizeAngle(aligned.theta - current.theta)) > kEps;
            current = aligned;
            if (moved || rotated)
            {
                poses.push_back(current);
            }

            remainingMove = scale(tangentDir, remainingAlong);
            remainingTheta = 0.0f;
        }

        return poses;
    }

    bool checkRobotCollision(
        const Pose2D& pose,
        const RobotConfig& robot,
        const std::vector<MapSegment>& walls)
    {
        for (const auto& wall : walls)
        {
            if (computeCollision(pose, robot, wall)) return true;
        }
        return false;
    }

    float raycastDistanceCm(
        float originX,
        float originY,
        float angleRad,
        float maxDistanceCm,
        const std::vector<MapSegment>& walls)
    {
        const float dx = std::cos(angleRad);
        const float dy = std::sin(angleRad);
        float bestT = maxDistanceCm;

        // Parametric ray-vs-segment intersection.
        // Ray:     (ox + t·dx, oy + t·dy), t ≥ 0
        // Segment: (sx + u·ex, sy + u·ey), u ∈ [0, 1]
        for (const auto& wall : walls)
        {
            const float ex = wall.endX - wall.startX;
            const float ey = wall.endY - wall.startY;
            const float denom = dx * ey - dy * ex;
            if (std::abs(denom) < kEps) continue;  // parallel

            const float sx = wall.startX - originX;
            const float sy = wall.startY - originY;
            const float t = (sx * ey - sy * ex) / denom;
            const float u = (sx * dy - sy * dx) / denom;
            if (t < 0.0f || t > bestT) continue;
            if (u < 0.0f || u > 1.0f) continue;
            bestT = t;
        }
        return bestT;
    }
}
