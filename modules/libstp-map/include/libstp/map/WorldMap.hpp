#pragma once

#include "libstp/map/Geometry.hpp"
#include "libstp/map/MapSegment.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace libstp::map
{
    /// Thrown by WorldMap parsers when the ftmap payload violates the
    /// flowchart-table-map v1 schema.
    class FtmapParseError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    /// Canonical Botball table map: bounds + line/wall segments + the
    /// geometric queries the particle filter and existing TableMap users
    /// need.
    ///
    /// Coordinate convention: origin at bottom-left of the table, +X right,
    /// +Y up, all values in cm. The ftmap on-disk format stores Y top-down;
    /// every parser in this class flips it on ingest so the in-memory
    /// representation stays bottom-up.
    ///
    /// This module supersedes both the in-sim WorldMap (libstp-sim still
    /// owns its copy until Phase 1 commit B migrates it) and the Python
    /// TableMap (deleted in a later commit). Sensor projection lives here
    /// so the Python side can call into it without paying FFI per particle.
    class WorldMap
    {
    public:
        WorldMap() = default;

        // ───────────── Bounds + segment access ─────────────
        float tableWidthCm() const noexcept { return m_tableWidthCm; }
        float tableHeightCm() const noexcept { return m_tableHeightCm; }
        const std::vector<MapSegment>& segments() const noexcept { return m_segments; }

        /// Subset of segments() with Kind::Line.
        std::vector<MapSegment> lines() const;

        /// Subset of segments() with Kind::Wall plus the four table border
        /// edges synthesized from the table bounds. The border edges have
        /// width=0 — `isOnWall` treats them as zero-thickness collision
        /// lines via the same min-threshold rule used for thin black lines.
        std::vector<MapSegment> walls() const;

        // ───────────── Point queries ─────────────

        /// Legacy alias — kept so callers porting from libstp::sim::WorldMap
        /// or the Python TableMap don't have to grep twice. Same logic as
        /// isOnLine.
        bool isOnBlackLine(float xCm, float yCm) const;

        /// True when (xCm, yCm) is within max(0.75 cm, widthCm/2) of any
        /// line segment. The 0.75 cm floor is the heuristic carried over
        /// from the web-ide TableMapService — thin authored lines still
        /// register as "on the line" for IR detection purposes.
        bool isOnLine(float xCm, float yCm) const;

        /// True when the point is within widthCm/2 of any wall segment OR
        /// within 0.5 cm of any of the four border edges. The border
        /// fallback is what makes "robot at the edge of the table" register
        /// as on-wall without needing the .ftmap to spell out border walls.
        bool isOnWall(float xCm, float yCm) const;

        /// Shortest distance from (xCm, yCm) to any line segment centerline,
        /// in cm. Returns +infinity when the map has no lines. Used by the
        /// particle filter as a soft observation likelihood.
        float distanceToNearestLine(float xCm, float yCm) const;

        /// Shortest distance from (xCm, yCm) to any wall segment OR border
        /// edge. Returns +infinity when the map has neither. Border edges
        /// are included so a robot just inside the table still gets a
        /// finite "distance to wall" reading.
        float distanceToNearestWall(float xCm, float yCm) const;

        // ───────────── Sensor projection ─────────────

        /// Project a sensor mounted at `sensor` (robot-local frame) into
        /// the field frame given the robot pose. forward_cm is along the
        /// heading, strafe_cm is 90° CCW. Equivalent to the body→world
        /// rotation used by raccoon.robot.table_map.sensor_field_position
        /// (kept bit-for-bit identical so the Python TableMap can be
        /// retired without behavior drift).
        Point2D sensorFieldPosition(float robotXCm,
                                    float robotYCm,
                                    float robotHeadingRad,
                                    const SensorOffset& sensor) const;

        /// Convenience: sensorFieldPosition + isOnLine.
        bool sensorIsOnLine(float robotXCm,
                            float robotYCm,
                            float robotHeadingRad,
                            const SensorOffset& sensor) const;

        /// Convenience: sensorFieldPosition + isOnWall.
        bool sensorIsOnWall(float robotXCm,
                            float robotYCm,
                            float robotHeadingRad,
                            const SensorOffset& sensor) const;

        // ───────────── Loading ─────────────

        /// Load an .ftmap file (flowchart-table-map v1, JSON-on-disk; we
        /// parse it via yaml-cpp because YAML is a JSON superset). Throws
        /// FtmapParseError for any schema violation.
        void loadFtmap(const std::filesystem::path& path);

        /// Parse an in-memory ftmap payload. Used by tests and by Python
        /// callers that already have the document as a string.
        void parseFtmap(const std::string& content);

        /// Replace the contents from a parsed Python ftmap dict (the same
        /// shape as TableMap.from_ftmap on the Python side). The C++
        /// binding extracts plain types from py::dict before calling here
        /// so this header stays pybind11-free.
        struct FtmapDictEntry
        {
            std::string kind;     // "line" or "wall"
            float startX{0.0f};
            float startY{0.0f};
            float endX{0.0f};
            float endY{0.0f};
            float widthCm{0.0f};
        };
        void loadFromDict(float tableWidthCm,
                          float tableHeightCm,
                          const std::vector<FtmapDictEntry>& entries);

        // ───────────── Programmatic construction ─────────────
        void setTable(float widthCm, float heightCm) noexcept
        {
            m_tableWidthCm = widthCm;
            m_tableHeightCm = heightCm;
        }
        void addSegment(const MapSegment& seg) { m_segments.push_back(seg); }
        void clear() noexcept
        {
            m_segments.clear();
            m_tableWidthCm = 0.0f;
            m_tableHeightCm = 0.0f;
        }

    private:
        float m_tableWidthCm{0.0f};
        float m_tableHeightCm{0.0f};
        std::vector<MapSegment> m_segments;
    };
}
