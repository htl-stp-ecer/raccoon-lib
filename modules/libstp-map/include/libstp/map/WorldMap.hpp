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

    /// One stacked 2D plane of the table (ground floor, raised ramp, …).
    /// Mirrors TableMapFileV2.layers[] from the web-ide ftmap schema. Each
    /// layer owns its line/wall segments in the shared table (x, y) frame at
    /// elevation `zCm`.
    struct MapLayer
    {
        std::string id;
        std::string name;
        float zCm{0.0f};
        std::vector<MapSegment> segments;
    };

    /// Edge geometry of a transition portal on one layer (cm, bottom-up frame).
    struct TransitionEdge
    {
        float startX{0.0f};
        float startY{0.0f};
        float endX{0.0f};
        float endY{0.0f};
    };

    /// A transition (ramp/portal) connecting two layers along a seam line on
    /// each. Crossing the `from` edge moves the robot onto `toLayer` (and back,
    /// when bidirectional). The parameter t along the edge maps 1:1 between
    /// layers; when the two edges are geometrically identical (a ramp drawn in
    /// the same table coordinates as the floor it rises from) the crossing is a
    /// pure layer switch with no position remap.
    struct MapTransition
    {
        std::string id;
        std::string name;
        std::string fromLayer;
        std::string toLayer;
        TransitionEdge from;
        TransitionEdge to;
        bool bidirectional{true};
        float widthCm{0.0f};
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

        // ───────────── Multi-layer (v2) access ─────────────
        // v1 maps load as a single "ground" layer; v2 maps load all authored
        // layers. `segments()` and the no-arg queries always operate on the
        // ground/active layer (m_segments) so the particle filter and every
        // pre-v2 caller keep working unchanged. SimWorld uses the layer-indexed
        // overloads with the plane the robot is currently on.

        std::size_t layerCount() const noexcept { return m_layers.size(); }
        const std::vector<MapLayer>& layers() const noexcept { return m_layers; }
        const std::vector<MapTransition>& transitions() const noexcept { return m_transitions; }

        /// Index of the layer with the given id, or -1 when absent.
        int layerIndex(const std::string& id) const noexcept;

        /// Segments of layer `idx`. Falls back to the ground segments when the
        /// map has no explicit layers or `idx` is out of range.
        const std::vector<MapSegment>& layerSegments(std::size_t idx) const noexcept;

        /// Layer-scoped variants of the point queries. Same semantics as the
        /// no-arg versions but against layer `idx`'s segments (table border
        /// walls still apply to every layer).
        bool isOnLine(float xCm, float yCm, std::size_t layerIdx) const;
        bool isOnWall(float xCm, float yCm, std::size_t layerIdx) const;

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

        /// Parse an in-memory ftmap payload (flowchart-table-map v1 or v2).
        /// Used by tests and by Python callers — including
        /// ``TableMap.from_ftmap``, whose binding serializes the project's
        /// ftmap dict to JSON and hands it here so there is one parser.
        void parseFtmap(const std::string& content);

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
            m_layers.clear();
            m_transitions.clear();
            m_tableWidthCm = 0.0f;
            m_tableHeightCm = 0.0f;
        }

    private:
        float m_tableWidthCm{0.0f};
        float m_tableHeightCm{0.0f};
        // Ground/active layer segments — backs segments() and the no-arg
        // queries. Mirrors m_layers[ground] so legacy callers are unaffected.
        std::vector<MapSegment> m_segments;
        std::vector<MapLayer> m_layers;
        std::vector<MapTransition> m_transitions;
    };
}
