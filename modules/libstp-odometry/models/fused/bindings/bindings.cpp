#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <memory>
#include <string>

#include "odometry/fused/fused_odometry.hpp"
#include "hal/IMU.hpp"
#include "kinematics/kinematics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(odometry_fused, m)
{
    m.doc() = "Python bindings for libstp-odometry_fused";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("raccoon.odometry");
    py::module_::import("raccoon.hal");
    py::module_::import("raccoon.kinematics");

    // DistanceFromOrigin struct
    py::class_<libstp::odometry::DistanceFromOrigin>(m, "DistanceFromOrigin",
            "Distance measurements from the origin (set by last reset).")
        .def_readonly("forward", &libstp::odometry::DistanceFromOrigin::forward,
                     "Distance in initial forward direction (m). Positive=forward, negative=backward.")
        .def_readonly("lateral", &libstp::odometry::DistanceFromOrigin::lateral,
                     "Lateral drift perpendicular to initial direction (m). Positive=right, negative=left.")
        .def_readonly("straight_line", &libstp::odometry::DistanceFromOrigin::straight_line,
                     "Straight-line Euclidean distance from origin (m).")
        .def("__repr__", [](const libstp::odometry::DistanceFromOrigin& d) {
            return "<DistanceFromOrigin forward=" + std::to_string(d.forward) +
                   " lateral=" + std::to_string(d.lateral) +
                   " straight_line=" + std::to_string(d.straight_line) + ">";
        });

    // FusedOdometryConfig struct
    py::class_<libstp::odometry::fused::FusedOdometryConfig>(m, "FusedOdometryConfig",
            "Configuration for FusedOdometry.")
        .def(py::init<int, bool, float, std::string>(),
             py::arg("imu_ready_timeout_ms") = 1000,
             py::arg("enable_accel_fusion") = true,
             py::arg("bemf_trust") = 0.95f,
             py::arg("turn_axis") = "world_z")
        .def_readwrite("imu_ready_timeout_ms", &libstp::odometry::fused::FusedOdometryConfig::imu_ready_timeout_ms,
                      "Timeout waiting for IMU to be ready (milliseconds, default: 1000)")
        .def_readwrite("enable_accel_fusion", &libstp::odometry::fused::FusedOdometryConfig::enable_accel_fusion,
                      "Enable complementary filter fusing BEMF + IMU accel (default: true)")
        .def_readwrite("bemf_trust", &libstp::odometry::fused::FusedOdometryConfig::bemf_trust,
                      "Complementary filter alpha: 1.0 = pure BEMF, 0.0 = pure IMU (default: 0.95)")
        .def_readwrite("turn_axis", &libstp::odometry::fused::FusedOdometryConfig::turn_axis,
                      "Yaw rate axis: 'world_z' (default), 'body_x', 'body_y', or 'body_z'. "
                      "Use a body axis when the robot's up-axis is not body Z.");

    py::class_<libstp::odometry::fused::FusedOdometry, libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::fused::FusedOdometry>>(
            m, "FusedOdometry",
            "Fused odometry combining IMU orientation with kinematics velocity integration.\n"
            "Integrates IMU orientation tracking with auto-calibration.\n"
            "Integrates body velocities from kinematics for position estimation.\n"
            "Tracks distances from origin in forward/lateral directions.\n"
            "Provides all coordinate frame transformations.\n"
            "Position will drift over time - use reset() to establish a fresh local origin.")
        // Shared ownership matches the C++ constructor signature for IMU and kinematics.
        .def(py::init<std::shared_ptr<libstp::hal::imu::IMU>,
                      std::shared_ptr<libstp::kinematics::IKinematics>,
                      libstp::odometry::fused::FusedOdometryConfig>(),
             py::arg("imu"),
             py::arg("kinematics"),
             py::arg("config") = libstp::odometry::fused::FusedOdometryConfig{},
             "Create fused odometry with IMU and kinematics model.\n\n"
             "IMU will be auto-calibrated on first update - the initial orientation\n"
             "is captured and all subsequent readings are relative to it.\n\n"
             "Args:\n"
             "    imu: IMU sensor instance (shared_ptr)\n"
             "    kinematics: Kinematics model (shared_ptr, provides velocity estimates)\n"
             "    config: Optional configuration (default uses 1000ms IMU timeout)")
        .def("update", &libstp::odometry::fused::FusedOdometry::update,
             py::arg("dt"),
             "Update odometry estimate with time delta.\n\n"
             "Args:\n"
             "    dt: Time delta in seconds")
        .def("get_pose", &libstp::odometry::fused::FusedOdometry::getPose,
             "Get current pose estimate (position + orientation).")
        .def("get_distance_from_origin", &libstp::odometry::fused::FusedOdometry::getDistanceFromOrigin,
             "Get distance measurements from origin (set by last reset).\n\n"
             "Returns:\n"
             "    DistanceFromOrigin with forward, lateral, and straight_line distances")
        .def("get_heading", &libstp::odometry::fused::FusedOdometry::getHeading,
             "Get current heading (yaw) relative to origin orientation.\n\n"
             "Returns:\n"
             "    Heading in radians, range [-π, π].\n"
             "    0 = facing initial forward direction\n"
             "    positive = rotated CCW, negative = rotated CW")
        .def("get_absolute_heading", &libstp::odometry::fused::FusedOdometry::getAbsoluteHeading,
             "Get absolute IMU heading unaffected by reset().\n\n"
             "Returns:\n"
             "    Heading in radians (CCW-positive). Stable across odometry resets.")
        .def("get_path_length", &libstp::odometry::fused::FusedOdometry::getPathLength,
             "Get cumulative path length (odometer) in meters.\n\n"
             "Monotonically increasing — NOT affected by reset().\n\n"
             "Returns:\n"
             "    Total distance traveled in meters since construction.")
        .def("get_heading_error", &libstp::odometry::fused::FusedOdometry::getHeadingError,
             py::arg("target_heading_rad"),
             "Get heading error to reach a target heading.\n\n"
             "Computes shortest angular path with proper wraparound handling.\n"
             "Perfect for use as PID input.\n\n"
             "Args:\n"
             "    target_heading_rad: Target heading in radians\n\n"
             "Returns:\n"
             "    Signed angular error in radians [-π, π].\n"
             "    Positive = turn CCW, negative = turn CW")
        .def("reset", py::overload_cast<>(&libstp::odometry::fused::FusedOdometry::reset),
             "Reset odometry to origin (zero position, identity orientation).\n\n"
             "IMU will be re-calibrated on next update.");
}
