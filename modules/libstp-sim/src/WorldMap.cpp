#include "libstp/sim/WorldMap.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace libstp::sim
{
    namespace
    {
        float pointToSegmentDistance(float x, float y, const MapSegment& seg)
        {
            const float vx = seg.endX - seg.startX;
            const float vy = seg.endY - seg.startY;
            const float wx = x - seg.startX;
            const float wy = y - seg.startY;

            const float c1 = vx * wx + vy * wy;
            if (c1 <= 0.0f)
            {
                return std::hypot(x - seg.startX, y - seg.startY);
            }

            const float c2 = vx * vx + vy * vy;
            if (c2 <= c1)
            {
                return std::hypot(x - seg.endX, y - seg.endY);
            }

            const float t = c1 / c2;
            const float px = seg.startX + t * vx;
            const float py = seg.startY + t * vy;
            return std::hypot(x - px, y - py);
        }
    }

    std::vector<MapSegment> WorldMap::lines() const
    {
        std::vector<MapSegment> out;
        out.reserve(m_segments.size());
        for (const auto& seg : m_segments)
        {
            if (seg.kind == MapSegment::Kind::Line) out.push_back(seg);
        }
        return out;
    }

    std::vector<MapSegment> WorldMap::walls() const
    {
        std::vector<MapSegment> out;
        for (const auto& seg : m_segments)
        {
            if (seg.kind == MapSegment::Kind::Wall) out.push_back(seg);
        }
        if (m_tableWidthCm > 0.0f && m_tableHeightCm > 0.0f)
        {
            out.push_back({MapSegment::Kind::Wall, 0.0f, 0.0f, m_tableWidthCm, 0.0f, 0.0f});
            out.push_back({MapSegment::Kind::Wall, m_tableWidthCm, 0.0f, m_tableWidthCm, m_tableHeightCm, 0.0f});
            out.push_back({MapSegment::Kind::Wall, m_tableWidthCm, m_tableHeightCm, 0.0f, m_tableHeightCm, 0.0f});
            out.push_back({MapSegment::Kind::Wall, 0.0f, m_tableHeightCm, 0.0f, 0.0f, 0.0f});
        }
        return out;
    }

    bool WorldMap::isOnBlackLine(float xCm, float yCm) const
    {
        for (const auto& seg : m_segments)
        {
            if (seg.kind != MapSegment::Kind::Line) continue;
            const float threshold = std::max(0.75f, seg.widthCm * 0.5f);
            if (pointToSegmentDistance(xCm, yCm, seg) <= threshold)
            {
                return true;
            }
        }
        return false;
    }

    void WorldMap::loadFtmap(const std::filesystem::path& path)
    {
        std::ifstream in(path);
        if (!in)
        {
            throw FtmapParseError("cannot open ftmap file: " + path.string());
        }
        std::stringstream buffer;
        buffer << in.rdbuf();
        parseFtmap(buffer.str());
    }

    void WorldMap::parseFtmap(const std::string& content)
    {
        YAML::Node root;
        try
        {
            root = YAML::Load(content);
        }
        catch (const YAML::Exception& e)
        {
            throw FtmapParseError(std::string{"ftmap parse error: "} + e.what());
        }

        if (!root.IsMap())
        {
            throw FtmapParseError("ftmap root must be an object");
        }

        const auto format = root["format"];
        if (!format || format.as<std::string>() != "flowchart-table-map")
        {
            throw FtmapParseError("ftmap format must be 'flowchart-table-map'");
        }

        const auto version = root["version"];
        if (!version || version.as<int>() != 1)
        {
            throw FtmapParseError("ftmap version must be 1");
        }

        const auto table = root["table"];
        if (!table || !table.IsMap())
        {
            throw FtmapParseError("ftmap.table missing");
        }

        clear();
        m_tableWidthCm = table["widthCm"].as<float>();
        m_tableHeightCm = table["heightCm"].as<float>();

        const auto lines = root["lines"];
        if (!lines)
        {
            return;
        }
        if (!lines.IsSequence())
        {
            throw FtmapParseError("ftmap.lines must be an array");
        }

        for (const auto& node : lines)
        {
            if (!node.IsMap())
            {
                throw FtmapParseError("ftmap.lines entry must be an object");
            }

            MapSegment seg{};
            const auto kind = node["kind"].as<std::string>();
            if (kind == "wall")
            {
                seg.kind = MapSegment::Kind::Wall;
            }
            else if (kind == "line")
            {
                seg.kind = MapSegment::Kind::Line;
            }
            else
            {
                throw FtmapParseError("ftmap.lines.kind must be 'line' or 'wall'");
            }

            // Flip Y to match the web-ide loadFromFtmap() convention: ftmap stores
            // Y with the origin at the top of the table, the sim uses Y with the
            // origin at the bottom.
            const float rawStartY = node["startY"].as<float>();
            const float rawEndY = node["endY"].as<float>();
            seg.startX = node["startX"].as<float>();
            seg.startY = m_tableHeightCm - rawStartY;
            seg.endX = node["endX"].as<float>();
            seg.endY = m_tableHeightCm - rawEndY;
            seg.widthCm = node["widthCm"].as<float>();

            m_segments.push_back(seg);
        }
    }
}
