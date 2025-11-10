//
// Created by tobias on 11/9/25.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "odometry/fused/fused_odometry.hpp"
#include "hal/IMU.hpp"
#include "kinematics/kinematics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(odometry_fused, m)
{
    m.doc() = "Python bindings for libstp-odometry_fused";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("libstp.odometry");
    py::module_::import("libstp.hal");
    py::module_::import("libstp.kinematics");

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

    py::class_<libstp::odometry::fused::FusedOdometry, libstp::odometry::IOdometry>(
            m, "FusedOdometry",
            "Fused odometry combining IMU orientation with kinematics velocity integration.\n"
            "Integrates IMU orientation tracking with auto-calibration.\n"
            "Integrates body velocities from kinematics for position estimation.\n"
            "Tracks distances from origin in forward/lateral directions.\n"
            "Provides all coordinate frame transformations.\n"
            "Position will drift over time - use reset() with external references to correct.")
        .def(py::init<libstp::hal::imu::IMU*, libstp::kinematics::IKinematics*>(),
             py::arg("imu"),
             py::arg("kinematics"),
             py::keep_alive<1, 2>(),  // Keep IMU alive
             py::keep_alive<1, 3>(),  // Keep kinematics alive
             "Create fused odometry with IMU and kinematics model.\n\n"
             "IMU will be auto-calibrated on first update - the initial orientation\n"
             "is captured and all subsequent readings are relative to it.\n\n"
             "Args:\n"
             "    imu: IMU sensor instance\n"
             "    kinematics: Kinematics model (provides velocity estimates)")
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
        .def("transform_to_body_frame", &libstp::odometry::fused::FusedOdometry::transformToBodyFrame,
             py::arg("world_vec"),
             "Transform vector from world frame to body frame.\n\n"
             "Args:\n"
             "    world_vec: 3D vector in world coordinates\n\n"
             "Returns:\n"
             "    Vector in body coordinates (forward=x, left=y, up=z)")
        .def("transform_to_world_frame", &libstp::odometry::fused::FusedOdometry::transformToWorldFrame,
             py::arg("body_vec"),
             "Transform vector from body frame to world frame.\n\n"
             "Args:\n"
             "    body_vec: 3D vector in body coordinates\n\n"
             "Returns:\n"
             "    Vector in world coordinates")
        .def("reset", py::overload_cast<const libstp::foundation::Pose&>(&libstp::odometry::fused::FusedOdometry::reset),
             py::arg("pose"),
             "Reset odometry to a specific pose.\n\n"
             "Sets new origin to this pose. IMU will be re-calibrated on next update.\n\n"
             "Args:\n"
             "    pose: Pose to reset to")
        .def("reset", py::overload_cast<>(&libstp::odometry::fused::FusedOdometry::reset),
             "Reset odometry to origin (zero position, identity orientation).\n\n"
             "IMU will be re-calibrated on next update.");
}
