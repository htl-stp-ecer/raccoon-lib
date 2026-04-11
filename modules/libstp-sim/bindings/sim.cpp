// Python bindings for libstp::sim.
//
// Installs as `raccoon.sim` — teams can drive SimWorld directly from pytest
// (e.g. attach sensors, tick by hand, assert pose) or attach it to the
// running MockPlatform singleton when the wheel is built with
// DRIVER_BUNDLE=mock. The mock helpers are compiled in only when MockPlatform
// is reachable, so wombat wheels still get the pure-geometry API.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "libstp/sim/Collision.hpp"
#include "libstp/sim/Pose2D.hpp"
#include "libstp/sim/RobotConfig.hpp"
#include "libstp/sim/SimWorld.hpp"
#include "libstp/sim/WorldMap.hpp"

#if __has_include("core/MockPlatform.hpp")
  #include "core/MockPlatform.hpp"
  #define RACCOON_SIM_HAS_MOCK 1
#endif

#include <optional>
#include <string>
#include <utility>

namespace py = pybind11;
using namespace libstp::sim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    float degToRad(float deg) { return deg * kPi / 180.0f; }
    float radToDeg(float rad) { return rad * 180.0f / kPi; }
}

PYBIND11_MODULE(sim, m)
{
    m.doc() = "libstp simulation bindings — pure 2D kinematic sim with walls, "
              "line sensors, and distance sensors. See scenes/README.md for "
              "the map file format.";

    // ───────────────────── Pose2D ─────────────────────
    py::class_<Pose2D>(m, "Pose2D")
        .def(py::init<>())
        .def(py::init([](float x, float y, float theta) { return Pose2D{x, y, theta}; }),
             py::arg("x"), py::arg("y"), py::arg("theta") = 0.0f,
             "Construct a pose. x,y in centimeters, theta in radians.")
        .def_static("from_degrees",
            [](float x, float y, float thetaDeg) { return Pose2D{x, y, degToRad(thetaDeg)}; },
            py::arg("x"), py::arg("y"), py::arg("theta_deg"),
            "Convenience constructor taking theta in degrees.")
        .def_readwrite("x", &Pose2D::x)
        .def_readwrite("y", &Pose2D::y)
        .def_readwrite("theta", &Pose2D::theta)
        .def_property_readonly("theta_deg",
            [](const Pose2D& p) { return radToDeg(p.theta); })
        .def("__repr__", [](const Pose2D& p) {
            return "Pose2D(x=" + std::to_string(p.x) +
                   ", y=" + std::to_string(p.y) +
                   ", theta=" + std::to_string(p.theta) + ")";
        });

    py::class_<Vec2>(m, "Vec2")
        .def(py::init<>())
        .def(py::init([](float x, float y) { return Vec2{x, y}; }),
             py::arg("x"), py::arg("y"))
        .def_readwrite("x", &Vec2::x)
        .def_readwrite("y", &Vec2::y);

    // ──────────────────── SensorMount ────────────────────
    py::class_<SensorMount>(m, "SensorMount")
        .def(py::init<>())
        .def_readwrite("name", &SensorMount::name)
        .def_readwrite("forward_cm", &SensorMount::forwardCm)
        .def_readwrite("strafe_cm", &SensorMount::strafeCm)
        .def_readwrite("analog_port", &SensorMount::analogPort)
        .def_readwrite("clearance_cm", &SensorMount::clearanceCm);

    // ──────────────────── RobotConfig ────────────────────
    py::class_<RobotConfig>(m, "RobotConfig")
        .def(py::init<>())
        .def_readwrite("width_cm", &RobotConfig::widthCm)
        .def_readwrite("length_cm", &RobotConfig::lengthCm)
        .def_readwrite("rotation_center_forward_cm", &RobotConfig::rotationCenterForwardCm)
        .def_readwrite("rotation_center_strafe_cm", &RobotConfig::rotationCenterStrafeCm)
        .def_readwrite("drive_type", &RobotConfig::driveType)
        .def_readwrite("wheel_radius_m", &RobotConfig::wheelRadiusM)
        .def_readwrite("track_width_m", &RobotConfig::trackWidthM)
        .def_readwrite("wheelbase_m", &RobotConfig::wheelbaseM)
        .def_readwrite("line_sensors", &RobotConfig::lineSensors)
        .def_readwrite("distance_sensors", &RobotConfig::distanceSensors);

    // ──────────────────── SimMotorMap ────────────────────
    py::class_<SimMotorMap>(m, "SimMotorMap")
        .def(py::init<>())
        .def_readwrite("left_port", &SimMotorMap::leftPort)
        .def_readwrite("right_port", &SimMotorMap::rightPort)
        .def_readwrite("left_inverted", &SimMotorMap::leftInverted)
        .def_readwrite("right_inverted", &SimMotorMap::rightInverted)
        .def_readwrite("max_wheel_velocity_rad_s", &SimMotorMap::maxWheelVelocityRadS)
        .def_readwrite("motor_time_constant_sec", &SimMotorMap::motorTimeConstantSec)
        .def_readwrite("ticks_to_rad", &SimMotorMap::ticksToRad);

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
        .def_readwrite("width_cm", &MapSegment::widthCm);

    // ──────────────────── WorldMap ────────────────────
    py::class_<WorldMap>(m, "WorldMap")
        .def(py::init<>())
        .def_property_readonly("table_width_cm", &WorldMap::tableWidthCm)
        .def_property_readonly("table_height_cm", &WorldMap::tableHeightCm)
        .def("set_table", &WorldMap::setTable, py::arg("width_cm"), py::arg("height_cm"))
        .def("add_segment", &WorldMap::addSegment)
        .def("clear", &WorldMap::clear)
        .def("segments", &WorldMap::segments)
        .def("lines", &WorldMap::lines)
        .def("walls", &WorldMap::walls)
        .def("is_on_black_line", &WorldMap::isOnBlackLine, py::arg("x_cm"), py::arg("y_cm"))
        .def("load_ftmap",
            [](WorldMap& self, const std::string& path) { self.loadFtmap(path); },
            py::arg("path"),
            "Load a .ftmap scene produced by the web-ide's map editor.")
        .def("parse_ftmap",
            [](WorldMap& self, const std::string& content) { self.parseFtmap(content); },
            py::arg("content"),
            "Parse ftmap content from an in-memory string.");

    py::register_exception<FtmapParseError>(m, "FtmapParseError");

    // ──────────────────── SimWorld ────────────────────
    py::class_<SimWorld>(m, "SimWorld")
        .def(py::init<>())
        .def("configure", &SimWorld::configure, py::arg("robot"), py::arg("motors"))
        .def("set_map", &SimWorld::setMap, py::arg("map"))
        .def_property_readonly("map", &SimWorld::map, py::return_value_policy::reference_internal)
        .def("set_pose", &SimWorld::setPose, py::arg("pose"))
        .def_property_readonly("pose", &SimWorld::pose)
        .def_property_readonly("yaw_rate_rad_s", &SimWorld::yawRateRadS)
        .def("set_motor_command", &SimWorld::setMotorCommand,
             py::arg("port"), py::arg("signed_percent"))
        .def("tick", &SimWorld::tick, py::arg("dt_seconds"))
        .def("attach_line_sensor", &SimWorld::attachLineSensor,
             py::arg("analog_port"), py::arg("forward_cm"), py::arg("strafe_cm"),
             py::arg("name") = std::string{})
        .def("attach_distance_sensor", &SimWorld::attachDistanceSensor,
             py::arg("analog_port"), py::arg("forward_cm"), py::arg("strafe_cm"),
             py::arg("mount_angle_rad") = 0.0f,
             py::arg("max_range_cm") = 100.0f,
             py::arg("name") = std::string{})
        .def("detach_sensor", &SimWorld::detachSensor, py::arg("analog_port"))
        .def("read_analog",
             [](const SimWorld& self, uint8_t port) -> py::object {
                 auto v = self.readAnalog(port);
                 if (!v) return py::none();
                 return py::int_(*v);
             },
             py::arg("analog_port"))
        .def("sensor_world_position", &SimWorld::sensorWorldPosition, py::arg("mount"))
        .def("is_sensor_on_black_line", &SimWorld::isSensorOnBlackLine, py::arg("mount"))
        .def_property_readonly("wheel_omega_left", &SimWorld::wheelOmegaLeft)
        .def_property_readonly("wheel_omega_right", &SimWorld::wheelOmegaRight)
        .def_property_readonly("trace", &SimWorld::trace)
        .def("clear_trace", &SimWorld::clearTrace);

    // ───────────── MockPlatform integration (mock bundle only) ─────────────
#if RACCOON_SIM_HAS_MOCK
    auto mockMod = m.def_submodule("mock",
        "Helpers for driving the process-wide MockPlatform singleton. Only "
        "present when the library is built with DRIVER_BUNDLE=mock.");

    mockMod.def("configure",
        [](const RobotConfig& robot, const SimMotorMap& motors,
           WorldMap map, const Pose2D& startPose) {
            platform::mock::core::MockPlatform::instance().configureSim(
                robot, motors, std::move(map), startPose);
        },
        py::arg("robot"), py::arg("motors"), py::arg("map"), py::arg("start_pose"),
        "Attach a fresh SimWorld to the MockPlatform singleton with the given "
        "config and scene. After this, HAL Motor::setSpeed writes drive the "
        "sim and OdometryBridge::readOdometry reports its pose.");

    mockMod.def("detach",
        [] { platform::mock::core::MockPlatform::instance().detachSim(); },
        "Detach the sim from MockPlatform. OdometryBridge goes back to "
        "reporting zero.");

    mockMod.def("has_sim",
        [] { return platform::mock::core::MockPlatform::instance().hasSim(); });

    mockMod.def("pose",
        [] { return platform::mock::core::MockPlatform::instance().simPose(); });

    mockMod.def("yaw_rate",
        [] { return platform::mock::core::MockPlatform::instance().simYawRate(); });

    mockMod.def("tick",
        [](float dt) { platform::mock::core::MockPlatform::instance().tickSim(dt); },
        py::arg("dt_seconds"),
        "Manually advance the sim by dt seconds. Use this from pytest for "
        "fully deterministic runs.");

    mockMod.def("enable_auto_tick",
        [](bool enabled) {
            platform::mock::core::MockPlatform::instance().setAutoTickEnabled(enabled);
        },
        py::arg("enabled"),
        "When enabled, every call into the mock HAL that reads sim state "
        "(getAnalog, readOdometry, gyroZ) first advances the sim by the "
        "wall-clock delta since the last tick.");

    mockMod.def("set_auto_tick_max_step",
        [](float maxDt) {
            platform::mock::core::MockPlatform::instance().setAutoTickMaxStep(maxDt);
        },
        py::arg("max_dt_seconds"),
        "Cap on how far a single auto-tick advances the sim (default 50 ms).");

    mockMod.def("attach_line_sensor",
        [](uint8_t port, float forwardCm, float strafeCm, std::string name) {
            auto* sim = platform::mock::core::MockPlatform::instance().sim();
            if (sim) sim->attachLineSensor(port, forwardCm, strafeCm, std::move(name));
        },
        py::arg("analog_port"), py::arg("forward_cm"), py::arg("strafe_cm"),
        py::arg("name") = std::string{});

    mockMod.def("attach_distance_sensor",
        [](uint8_t port, float forwardCm, float strafeCm,
           float mountAngleRad, float maxRangeCm, std::string name) {
            auto* sim = platform::mock::core::MockPlatform::instance().sim();
            if (sim) sim->attachDistanceSensor(
                port, forwardCm, strafeCm, mountAngleRad, maxRangeCm, std::move(name));
        },
        py::arg("analog_port"), py::arg("forward_cm"), py::arg("strafe_cm"),
        py::arg("mount_angle_rad") = 0.0f,
        py::arg("max_range_cm") = 100.0f,
        py::arg("name") = std::string{});

    mockMod.def("set_motor_command",
        [](uint8_t port, int signedPercent) {
            // Emulates what the HAL Motor::setSpeed path does on the mock
            // platform: picks a direction and duty and delegates to setMotor.
            // Lets pytest drive the sim motors without needing the full HAL
            // Python bindings or an installed raccoon package.
            using platform::mock::core::MockPlatform;
            using platform::mock::core::MotorDir;
            MotorDir dir = MotorDir::Off;
            if (signedPercent > 0) dir = MotorDir::CW;
            else if (signedPercent < 0) dir = MotorDir::CCW;
            const uint32_t duty = static_cast<uint32_t>(std::abs(signedPercent) * 4);
            MockPlatform::instance().setMotor(port, dir, duty);
        },
        py::arg("port"), py::arg("signed_percent"),
        "Write a motor command through the MockPlatform HAL path — the same "
        "one the real Motor::setSpeed uses. This drives the attached SimWorld.");

    mockMod.def("read_odometry",
        [] {
            // Mirrors what libstp::hal::odometry_bridge::OdometryBridge::
            // readOdometry() returns: pose is relative to the last
            // resetSimOrigin() / configureSim() call, in meters and radians.
            // Yaw rate is the live ground-truth angular velocity.
            auto& p = platform::mock::core::MockPlatform::instance();
            p.autoTickIfEnabled();
            const auto rel = p.simRelativePose();
            return py::make_tuple(rel.x * 0.01f, rel.y * 0.01f,
                                  rel.theta, p.simYawRate());
        },
        "Return the current odometry snapshot as (x_m, y_m, heading_rad, "
        "yaw_rate_rad_s), relative to the last reset / configure. Auto-ticks "
        "if enabled.");

    mockMod.def("reset_origin",
        [] { platform::mock::core::MockPlatform::instance().resetSimOrigin(); },
        "Capture the current ground-truth pose as the new odometry origin. "
        "After this, read_odometry() reports motion relative to the captured "
        "pose. Mirrors STM32 coprocessor reset behavior.");

    mockMod.def("read_analog",
        [](uint8_t port) {
            return platform::mock::core::MockPlatform::instance().getAnalog(port);
        },
        py::arg("port"),
        "Read an analog value through the mock HAL path. Auto-ticks if "
        "enabled and returns the sim sensor value when one is bound to "
        "`port`, otherwise the stock static analog value.");
#endif
}
