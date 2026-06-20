// Python bindings for libstp::map.
//
// Installs as `raccoon.map` — the canonical Python entry point for table
// geometry and sensor projection. Replaces the pure-Python TableMap (Phase 1
// of the absolute-motion plan deletes that file in a later commit; for now
// both coexist so libstp-sim and downstream callers can migrate
// independently).

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "libstp/map/Geometry.hpp"
#include "libstp/map/MapSegment.hpp"
#include "libstp/map/WorldMap.hpp"

#include <string>
#include <vector>

namespace py = pybind11;
using namespace libstp::map;

namespace
{
    // Pull a SensorOffset out of either a real SensorOffset (fast path) or
    // any Python object that exposes ``forward_cm`` / ``strafe_cm`` floats.
    // The duck-typed branch is what lets the Python side keep using the
    // existing SensorPosition dataclass (and any user-defined replacement)
    // without forcing a binding-side conversion.
    SensorOffset coerceSensor(const py::object& obj)
    {
        // Direct cast first — covers callers that already constructed a
        // SensorOffset.
        try
        {
            return obj.cast<SensorOffset>();
        }
        catch (const py::cast_error&)
        {
            // fall through to attribute-based extraction
        }

        SensorOffset out{};
        if (!py::hasattr(obj, "forward_cm") || !py::hasattr(obj, "strafe_cm"))
        {
            throw py::type_error(
                "sensor argument must be a SensorOffset or expose forward_cm/strafe_cm attributes");
        }
        out.forwardCm = obj.attr("forward_cm").cast<float>();
        out.strafeCm = obj.attr("strafe_cm").cast<float>();
        return out;
    }

    // Convert a Python ftmap dict (TableMap.from_ftmap shape) into the
    // dict-entry vector the C++ side expects. Validates the format/version
    // and raises FtmapParseError on mismatch — same surface as parseFtmap.
    void applyFtmapDict(WorldMap& map, const py::dict& data)
    {
        auto get = [&](const char* key) -> py::object {
            if (!data.contains(key))
            {
                throw FtmapParseError(std::string{"ftmap missing key: "} + key);
            }
            return data[key];
        };

        const auto fmt = get("format").cast<std::string>();
        if (fmt != "flowchart-table-map")
        {
            throw FtmapParseError("ftmap format must be 'flowchart-table-map'");
        }
        const auto version = get("version").cast<int>();
        if (version != 1)
        {
            throw FtmapParseError("ftmap version must be 1");
        }

        const auto tableObj = get("table");
        if (!py::isinstance<py::dict>(tableObj))
        {
            throw FtmapParseError("ftmap.table must be an object");
        }
        const auto table = tableObj.cast<py::dict>();
        const float widthCm = table["widthCm"].cast<float>();
        const float heightCm = table["heightCm"].cast<float>();

        std::vector<WorldMap::FtmapDictEntry> entries;
        if (data.contains("lines"))
        {
            for (auto item : data["lines"].cast<py::list>())
            {
                auto entry = item.cast<py::dict>();
                WorldMap::FtmapDictEntry e{};
                e.kind = entry.contains("kind") ? entry["kind"].cast<std::string>() : "line";
                e.startX = entry["startX"].cast<float>();
                e.startY = entry["startY"].cast<float>();
                e.endX = entry["endX"].cast<float>();
                e.endY = entry["endY"].cast<float>();
                e.widthCm = entry["widthCm"].cast<float>();
                entries.push_back(std::move(e));
            }
        }

        map.loadFromDict(widthCm, heightCm, entries);
    }
}

PYBIND11_MODULE(map, m)
{
    m.doc() = "Canonical raccoon table-map module (libstp::map). Provides "
              "WorldMap, MapSegment, and sensor-projection queries used by "
              "the localization stack and Python TableMap consumers.";

    py::register_exception<FtmapParseError>(m, "FtmapParseError");

    // ──────────────────── Geometry POD types ────────────────────
    py::class_<SensorOffset>(m, "SensorOffset",
        "Sensor mount offset from the robot geometric center. forward_cm is "
        "along the robot heading, strafe_cm is 90° CCW (positive = robot's "
        "left).")
        .def(py::init<>())
        .def(py::init([](float forward_cm, float strafe_cm) {
                 return SensorOffset{forward_cm, strafe_cm};
             }),
             py::arg("forward_cm"), py::arg("strafe_cm"))
        .def_readwrite("forward_cm", &SensorOffset::forwardCm)
        .def_readwrite("strafe_cm", &SensorOffset::strafeCm)
        .def("__repr__", [](const SensorOffset& s) {
            return "SensorOffset(forward_cm=" + std::to_string(s.forwardCm) +
                   ", strafe_cm=" + std::to_string(s.strafeCm) + ")";
        });

    py::class_<WheelOffset>(m, "WheelOffset",
        "Wheel mount offset from the robot geometric center. Same axes as "
        "SensorOffset.")
        .def(py::init<>())
        .def(py::init([](float forward_cm, float strafe_cm) {
                 return WheelOffset{forward_cm, strafe_cm};
             }),
             py::arg("forward_cm"), py::arg("strafe_cm"))
        .def_readwrite("forward_cm", &WheelOffset::forwardCm)
        .def_readwrite("strafe_cm", &WheelOffset::strafeCm);

    py::class_<Point2D>(m, "Point2D",
        "Point in field coordinates (cm, origin = bottom-left, +X right, +Y up).")
        .def(py::init<>())
        .def(py::init([](float x_cm, float y_cm) { return Point2D{x_cm, y_cm}; }),
             py::arg("x_cm"), py::arg("y_cm"))
        .def_readwrite("x_cm", &Point2D::xCm)
        .def_readwrite("y_cm", &Point2D::yCm)
        .def("__iter__", [](const Point2D& p) {
            // Allows ``x, y = sensor_field_position(...)`` like the Python
            // TableMap which returned a tuple.
            return py::iter(py::make_tuple(p.xCm, p.yCm));
        })
        .def("__getitem__", [](const Point2D& p, size_t i) {
            if (i == 0) return p.xCm;
            if (i == 1) return p.yCm;
            throw py::index_error("Point2D index out of range");
        })
        .def("__repr__", [](const Point2D& p) {
            return "Point2D(x_cm=" + std::to_string(p.xCm) +
                   ", y_cm=" + std::to_string(p.yCm) + ")";
        });

    // ──────────────────── MapSegment ────────────────────
    py::class_<MapSegment> mapSeg(m, "MapSegment");
    py::enum_<MapSegment::Kind>(mapSeg, "Kind")
        .value("LINE", MapSegment::Kind::Line)
        .value("WALL", MapSegment::Kind::Wall);
    mapSeg
        .def(py::init<>())
        .def_readwrite("kind", &MapSegment::kind)
        .def_readwrite("start_x", &MapSegment::startX)
        .def_readwrite("start_y", &MapSegment::startY)
        .def_readwrite("end_x", &MapSegment::endX)
        .def_readwrite("end_y", &MapSegment::endY)
        .def_readwrite("width_cm", &MapSegment::widthCm)
        .def_property_readonly("length", &MapSegment::length);

    // ──────────────────── WorldMap ────────────────────
    py::class_<WorldMap>(m, "WorldMap",
        "Canonical Botball table map. Bottom-left origin, +X right, +Y up, "
        "all dimensions in cm.")
        .def(py::init<>())
        // Bounds — both legacy short names (width_cm/height_cm, used by the
        // Python TableMap) and the libstp-sim names (table_width_cm/
        // table_height_cm) are exposed so neither call site has to change.
        .def_property_readonly("table_width_cm", &WorldMap::tableWidthCm)
        .def_property_readonly("table_height_cm", &WorldMap::tableHeightCm)
        .def_property_readonly("width_cm", &WorldMap::tableWidthCm)
        .def_property_readonly("height_cm", &WorldMap::tableHeightCm)

        .def("set_table", &WorldMap::setTable, py::arg("width_cm"), py::arg("height_cm"))
        .def("add_segment", &WorldMap::addSegment, py::arg("segment"))
        .def("clear", &WorldMap::clear)

        .def("segments", &WorldMap::segments)
        .def("lines", &WorldMap::lines)
        .def("walls", &WorldMap::walls,
             "Wall segments plus the four synthesized table-border edges.")
        // ``all_segments`` mirrors the legacy Python TableMap property —
        // every authored segment (lines + explicit walls) without the
        // synthesized table-border walls. ``lines()``/``walls()`` above stay
        // method-style; existing call sites already use the method form.
        .def_property_readonly("all_segments", &WorldMap::segments,
             "Authored segments (lines + explicit walls), excluding the "
             "synthesized table-border walls.")

        .def("is_on_line",
             [](const WorldMap& self, float x_cm, float y_cm) {
                 return self.isOnLine(x_cm, y_cm);
             },
             py::arg("x_cm"), py::arg("y_cm"))
        // ── Multi-layer (v2) access ──
        .def_property_readonly("layer_count",
             [](const WorldMap& self) { return self.layerCount(); })
        .def("layer_index", &WorldMap::layerIndex, py::arg("id"),
             "Index of the layer with this id, or -1 when absent.")
        .def_property_readonly("transition_count",
             [](const WorldMap& self) { return self.transitions().size(); })
        .def("layer_segments",
             [](const WorldMap& self, std::size_t idx) { return self.layerSegments(idx); },
             py::arg("layer_idx"), "Authored segments of layer `layer_idx`.")
        .def("is_on_line_layer",
             [](const WorldMap& self, float x_cm, float y_cm, std::size_t layer_idx) {
                 return self.isOnLine(x_cm, y_cm, layer_idx);
             },
             py::arg("x_cm"), py::arg("y_cm"), py::arg("layer_idx"),
             "is_on_line resolved against a specific layer's segments.")
        .def("is_on_black_line", &WorldMap::isOnBlackLine,
             py::arg("x_cm"), py::arg("y_cm"),
             "Legacy alias for is_on_line, kept for libstp-sim compatibility.")
        .def("is_on_wall",
             [](const WorldMap& self, float x_cm, float y_cm) {
                 return self.isOnWall(x_cm, y_cm);
             },
             py::arg("x_cm"), py::arg("y_cm"))
        .def("distance_to_nearest_line", &WorldMap::distanceToNearestLine,
             py::arg("x_cm"), py::arg("y_cm"))
        .def("distance_to_nearest_wall", &WorldMap::distanceToNearestWall,
             py::arg("x_cm"), py::arg("y_cm"))

        .def("sensor_field_position",
             [](const WorldMap& self, float robot_x_cm, float robot_y_cm,
                float robot_heading_rad, const py::object& sensor) {
                 return self.sensorFieldPosition(robot_x_cm, robot_y_cm,
                                                 robot_heading_rad,
                                                 coerceSensor(sensor));
             },
             py::arg("robot_x_cm"), py::arg("robot_y_cm"),
             py::arg("robot_heading_rad"), py::arg("sensor"),
             "Project a robot-local sensor mount into field coordinates. "
             "`sensor` may be a SensorOffset or any object with "
             "`forward_cm`/`strafe_cm` attributes (e.g. SensorPosition).")
        .def("sensor_is_on_line",
             [](const WorldMap& self, float robot_x_cm, float robot_y_cm,
                float robot_heading_rad, const py::object& sensor) {
                 return self.sensorIsOnLine(robot_x_cm, robot_y_cm,
                                            robot_heading_rad,
                                            coerceSensor(sensor));
             },
             py::arg("robot_x_cm"), py::arg("robot_y_cm"),
             py::arg("robot_heading_rad"), py::arg("sensor"))
        .def("sensor_is_on_wall",
             [](const WorldMap& self, float robot_x_cm, float robot_y_cm,
                float robot_heading_rad, const py::object& sensor) {
                 return self.sensorIsOnWall(robot_x_cm, robot_y_cm,
                                            robot_heading_rad,
                                            coerceSensor(sensor));
             },
             py::arg("robot_x_cm"), py::arg("robot_y_cm"),
             py::arg("robot_heading_rad"), py::arg("sensor"))

        // ─── Loading paths ───
        .def("load",
             [](WorldMap& self, const std::string& path) { self.loadFtmap(path); },
             py::arg("path"),
             "Load an .ftmap file from disk (TableMapFileV1 schema).")
        .def("load_ftmap",
             [](WorldMap& self, const std::string& path) { self.loadFtmap(path); },
             py::arg("path"),
             "Legacy alias for load(), kept for libstp-sim compatibility.")
        .def("parse_ftmap",
             [](WorldMap& self, const std::string& content) { self.parseFtmap(content); },
             py::arg("content"),
             "Parse an in-memory ftmap document.")
        .def_static("from_ftmap",
            [](const py::dict& data) {
                WorldMap m;
                applyFtmapDict(m, data);
                return m;
            },
            py::arg("data"),
            "Build a WorldMap from a parsed ftmap dict (the same shape that "
            "raccoon.project.yml stores under robot.physical.table_map).")

        .def("__repr__", [](const WorldMap& m) {
            // Match the legacy Python TableMap repr so log lines and
            // assertions that grew up around it keep working.
            const auto lines = m.lines();
            const auto walls = m.walls();
            return "TableMap(" + std::to_string(m.tableWidthCm()) + "x" +
                   std::to_string(m.tableHeightCm()) + "cm, " +
                   std::to_string(lines.size()) + " lines, " +
                   std::to_string(walls.size()) + " walls)";
        });
}
