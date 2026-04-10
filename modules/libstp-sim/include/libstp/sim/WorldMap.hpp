#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace libstp::sim
{
    /// One line or wall segment in table coordinates (cm).
    /// Ports TableMapFileV1.lines[] from the web-ide ftmap schema.
    struct MapSegment
    {
        enum class Kind : uint8_t { Line, Wall };

        Kind kind{Kind::Line};
        float startX{0.0f};
        float startY{0.0f};
        float endX{0.0f};
        float endY{0.0f};
        float widthCm{0.0f};
    };

    class FtmapParseError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    /// The simulation world: table bounds + black lines + walls.
    /// Ports the geometric queries from web-ide TableMapService.
    class WorldMap
    {
    public:
        WorldMap() = default;

        float tableWidthCm() const noexcept { return m_tableWidthCm; }
        float tableHeightCm() const noexcept { return m_tableHeightCm; }
        const std::vector<MapSegment>& segments() const noexcept { return m_segments; }

        /// Return only segments with Kind::Line.
        std::vector<MapSegment> lines() const;

        /// Return Kind::Wall segments plus the four table border edges.
        /// Equivalent to web-ide buildCollisionWalls() output.
        std::vector<MapSegment> walls() const;

        /// Port of web-ide TableMapService::isOnBlackLine. Pure geometric:
        /// returns true if (xCm, yCm) is within max(0.75 cm, thickness/2) of
        /// any line segment.
        bool isOnBlackLine(float xCm, float yCm) const;

        /// Load from an ftmap file (TableMapFileV1 schema). Throws
        /// FtmapParseError on any schema violation.
        void loadFtmap(const std::filesystem::path& path);

        /// Parse ftmap content from an in-memory string. Useful for tests and
        /// for Python bindings that pass scene content directly.
        void parseFtmap(const std::string& content);

        /// Set contents directly (used by tests and by programmatic scenes).
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
