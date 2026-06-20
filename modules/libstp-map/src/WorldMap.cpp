#include "libstp/map/WorldMap.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace libstp::map
{
    namespace
    {
        // Min detection threshold for "is on line" — matches the web-ide
        // TableMapService heuristic and the existing libstp::sim::WorldMap
        // copy. Keep the constant here so the binding tests can rely on the
        // same number.
        constexpr float kMinLineThresholdCm = 0.75f;
        // Tolerance for "is on (border) wall". Border edges have width 0,
        // so without a floor the predicate would only fire for points
        // exactly on the boundary, which is useless for IR sensing.
        constexpr float kBorderWallToleranceCm = 0.5f;

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

        std::array<MapSegment, 4> borderEdges(float w, float h)
        {
            // Counterclockwise from origin. width=0 — they're treated as
            // zero-thickness collision lines.
            return {
                MapSegment{MapSegment::Kind::Wall, 0.0f, 0.0f, w, 0.0f, 0.0f},
                MapSegment{MapSegment::Kind::Wall, w, 0.0f, w, h, 0.0f},
                MapSegment{MapSegment::Kind::Wall, w, h, 0.0f, h, 0.0f},
                MapSegment{MapSegment::Kind::Wall, 0.0f, h, 0.0f, 0.0f, 0.0f},
            };
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
            for (const auto& edge : borderEdges(m_tableWidthCm, m_tableHeightCm))
            {
                out.push_back(edge);
            }
        }
        return out;
    }

    bool WorldMap::isOnLine(float xCm, float yCm) const
    {
        for (const auto& seg : m_segments)
        {
            if (seg.kind != MapSegment::Kind::Line) continue;
            const float threshold = std::max(kMinLineThresholdCm, seg.widthCm * 0.5f);
            if (pointToSegmentDistance(xCm, yCm, seg) <= threshold)
            {
                return true;
            }
        }
        return false;
    }

    bool WorldMap::isOnBlackLine(float xCm, float yCm) const
    {
        return isOnLine(xCm, yCm);
    }

    bool WorldMap::isOnWall(float xCm, float yCm) const
    {
        for (const auto& seg : m_segments)
        {
            if (seg.kind != MapSegment::Kind::Wall) continue;
            const float threshold = std::max(kBorderWallToleranceCm, seg.widthCm * 0.5f);
            if (pointToSegmentDistance(xCm, yCm, seg) <= threshold)
            {
                return true;
            }
        }
        if (m_tableWidthCm > 0.0f && m_tableHeightCm > 0.0f)
        {
            for (const auto& edge : borderEdges(m_tableWidthCm, m_tableHeightCm))
            {
                if (pointToSegmentDistance(xCm, yCm, edge) <= kBorderWallToleranceCm)
                {
                    return true;
                }
            }
        }
        return false;
    }

    int WorldMap::layerIndex(const std::string& id) const noexcept
    {
        for (std::size_t i = 0; i < m_layers.size(); ++i)
        {
            if (m_layers[i].id == id) return static_cast<int>(i);
        }
        return -1;
    }

    const std::vector<MapSegment>& WorldMap::layerSegments(std::size_t idx) const noexcept
    {
        if (idx < m_layers.size())
        {
            return m_layers[idx].segments;
        }
        return m_segments;  // v1 / out-of-range → ground segments
    }

    bool WorldMap::isOnLine(float xCm, float yCm, std::size_t layerIdx) const
    {
        for (const auto& seg : layerSegments(layerIdx))
        {
            if (seg.kind != MapSegment::Kind::Line) continue;
            const float threshold = std::max(kMinLineThresholdCm, seg.widthCm * 0.5f);
            if (pointToSegmentDistance(xCm, yCm, seg) <= threshold)
            {
                return true;
            }
        }
        return false;
    }

    bool WorldMap::isOnWall(float xCm, float yCm, std::size_t layerIdx) const
    {
        for (const auto& seg : layerSegments(layerIdx))
        {
            if (seg.kind != MapSegment::Kind::Wall) continue;
            const float threshold = std::max(kBorderWallToleranceCm, seg.widthCm * 0.5f);
            if (pointToSegmentDistance(xCm, yCm, seg) <= threshold)
            {
                return true;
            }
        }
        if (m_tableWidthCm > 0.0f && m_tableHeightCm > 0.0f)
        {
            for (const auto& edge : borderEdges(m_tableWidthCm, m_tableHeightCm))
            {
                if (pointToSegmentDistance(xCm, yCm, edge) <= kBorderWallToleranceCm)
                {
                    return true;
                }
            }
        }
        return false;
    }

    float WorldMap::distanceToNearestLine(float xCm, float yCm) const
    {
        float best = std::numeric_limits<float>::infinity();
        for (const auto& seg : m_segments)
        {
            if (seg.kind != MapSegment::Kind::Line) continue;
            best = std::min(best, pointToSegmentDistance(xCm, yCm, seg));
        }
        return best;
    }

    float WorldMap::distanceToNearestWall(float xCm, float yCm) const
    {
        float best = std::numeric_limits<float>::infinity();
        for (const auto& seg : m_segments)
        {
            if (seg.kind != MapSegment::Kind::Wall) continue;
            best = std::min(best, pointToSegmentDistance(xCm, yCm, seg));
        }
        if (m_tableWidthCm > 0.0f && m_tableHeightCm > 0.0f)
        {
            for (const auto& edge : borderEdges(m_tableWidthCm, m_tableHeightCm))
            {
                best = std::min(best, pointToSegmentDistance(xCm, yCm, edge));
            }
        }
        return best;
    }

    Point2D WorldMap::sensorFieldPosition(float robotXCm,
                                          float robotYCm,
                                          float robotHeadingRad,
                                          const SensorOffset& sensor) const
    {
        const float cosH = std::cos(robotHeadingRad);
        const float sinH = std::sin(robotHeadingRad);
        // forward along heading; strafe rotated 90° CCW from heading.
        // Identical to raccoon.robot.table_map.sensor_field_position.
        Point2D out{};
        out.xCm = robotXCm + sensor.forwardCm * cosH - sensor.strafeCm * sinH;
        out.yCm = robotYCm + sensor.forwardCm * sinH + sensor.strafeCm * cosH;
        return out;
    }

    bool WorldMap::sensorIsOnLine(float robotXCm,
                                  float robotYCm,
                                  float robotHeadingRad,
                                  const SensorOffset& sensor) const
    {
        const auto p = sensorFieldPosition(robotXCm, robotYCm, robotHeadingRad, sensor);
        return isOnLine(p.xCm, p.yCm);
    }

    bool WorldMap::sensorIsOnWall(float robotXCm,
                                  float robotYCm,
                                  float robotHeadingRad,
                                  const SensorOffset& sensor) const
    {
        const auto p = sensorFieldPosition(robotXCm, robotYCm, robotHeadingRad, sensor);
        return isOnWall(p.xCm, p.yCm);
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
        const int ver = version ? version.as<int>() : 0;
        if (ver != 1 && ver != 2)
        {
            throw FtmapParseError("ftmap version must be 1 or 2");
        }

        const auto table = root["table"];
        if (!table || !table.IsMap())
        {
            throw FtmapParseError("ftmap.table missing");
        }

        clear();
        m_tableWidthCm = table["widthCm"].as<float>();
        m_tableHeightCm = table["heightCm"].as<float>();

        // Parse one line/wall node. ftmap stores Y top-down; we live bottom-up,
        // so flip on ingest (shared by v1 lines[] and v2 layers[].lines[]).
        const float h = m_tableHeightCm;
        auto parseSegment = [h](const YAML::Node& node) -> MapSegment {
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
            seg.startX = node["startX"].as<float>();
            seg.startY = h - node["startY"].as<float>();
            seg.endX = node["endX"].as<float>();
            seg.endY = h - node["endY"].as<float>();
            seg.widthCm = node["widthCm"].as<float>();
            return seg;
        };

        if (ver == 2)
        {
            const auto layers = root["layers"];
            if (!layers || !layers.IsSequence())
            {
                throw FtmapParseError("ftmap v2 requires a 'layers' array");
            }
            for (const auto& ln : layers)
            {
                if (!ln.IsMap())
                {
                    throw FtmapParseError("ftmap.layers entry must be an object");
                }
                MapLayer layer{};
                layer.id = ln["id"] ? ln["id"].as<std::string>() : std::string{};
                layer.name = ln["name"] ? ln["name"].as<std::string>() : layer.id;
                layer.zCm = ln["zCm"] ? ln["zCm"].as<float>() : 0.0f;
                if (const auto llines = ln["lines"]; llines && llines.IsSequence())
                {
                    for (const auto& node : llines)
                    {
                        layer.segments.push_back(parseSegment(node));
                    }
                }
                m_layers.push_back(std::move(layer));
            }

            // The ground/active layer backs the legacy no-arg API + the particle
            // filter. Prefer an explicit "ground" id, else the first layer.
            std::size_t activeIdx = 0;
            if (const int groundIdx = layerIndex("ground"); groundIdx >= 0)
            {
                activeIdx = static_cast<std::size_t>(groundIdx);
            }
            if (!m_layers.empty())
            {
                m_segments = m_layers[activeIdx].segments;
            }

            auto parseEdge = [h](const YAML::Node& e) -> TransitionEdge {
                TransitionEdge te{};
                te.startX = e["startX"].as<float>();
                te.startY = h - e["startY"].as<float>();
                te.endX = e["endX"].as<float>();
                te.endY = h - e["endY"].as<float>();
                return te;
            };
            if (const auto trans = root["transitions"]; trans && trans.IsSequence())
            {
                for (const auto& tn : trans)
                {
                    MapTransition t{};
                    t.id = tn["id"] ? tn["id"].as<std::string>() : std::string{};
                    t.name = tn["name"] ? tn["name"].as<std::string>() : std::string{};
                    t.fromLayer = tn["fromLayer"].as<std::string>();
                    t.toLayer = tn["toLayer"].as<std::string>();
                    t.from = parseEdge(tn["from"]);
                    t.to = parseEdge(tn["to"]);
                    t.bidirectional = tn["bidirectional"] ? tn["bidirectional"].as<bool>() : true;
                    t.widthCm = tn["widthCm"] ? tn["widthCm"].as<float>() : 0.0f;
                    m_transitions.push_back(std::move(t));
                }
            }
            return;
        }

        // ---- v1: single implicit "ground" layer ----
        if (const auto lines = root["lines"])
        {
            if (!lines.IsSequence())
            {
                throw FtmapParseError("ftmap.lines must be an array");
            }
            for (const auto& node : lines)
            {
                m_segments.push_back(parseSegment(node));
            }
        }
        m_layers.push_back(MapLayer{"ground", "Ground", 0.0f, m_segments});
    }

    void WorldMap::loadFromDict(float tableWidthCm,
                                float tableHeightCm,
                                const std::vector<FtmapDictEntry>& entries)
    {
        clear();
        m_tableWidthCm = tableWidthCm;
        m_tableHeightCm = tableHeightCm;

        for (const auto& e : entries)
        {
            MapSegment seg{};
            if (e.kind == "wall")
            {
                seg.kind = MapSegment::Kind::Wall;
            }
            else if (e.kind == "line")
            {
                seg.kind = MapSegment::Kind::Line;
            }
            else
            {
                throw FtmapParseError("ftmap.lines.kind must be 'line' or 'wall'");
            }

            // Same Y-flip as parseFtmap — keeps both ingest paths
            // semantically identical so a pytest that loads a JSON dict and
            // a C++ test that loads the YAML string produce the same map.
            seg.startX = e.startX;
            seg.startY = tableHeightCm - e.startY;
            seg.endX = e.endX;
            seg.endY = tableHeightCm - e.endY;
            seg.widthCm = e.widthCm;
            m_segments.push_back(seg);
        }
        m_layers.push_back(MapLayer{"ground", "Ground", 0.0f, m_segments});
    }
}
