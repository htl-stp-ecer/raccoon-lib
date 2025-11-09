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
    py::module_::import("libstp.odometry_imu");  // Import ImuOdometry module for ImuOdometryConfig
    py::module_::import("libstp.hal");
    py::module_::import("libstp.kinematics");

    py::class_<libstp::odometry::fused::FusedOdometry, libstp::odometry::IOdometry>(
            m, "FusedOdometry",
            "Fused odometry combining IMU orientation with kinematics velocity integration.\n"
            "Delegates orientation tracking to ImuOdometry (handles IMU inversions, initialization).\n"
            "Integrates body velocities from kinematics for position estimation.\n"
            "Position will drift over time - use reset() with external references to correct.")
        .def(py::init([](libstp::hal::imu::IMU* imu,
                         libstp::kinematics::IKinematics* kinematics,
                         bool invert_x,
                         bool invert_y,
                         bool invert_z,
                         bool invert_w) {
                 libstp::odometry::imu::ImuOdometryConfig config{};
                 config.invert_x = invert_x;
                 config.invert_y = invert_y;
                 config.invert_z = invert_z;
                 config.invert_w = invert_w;
                 return new libstp::odometry::fused::FusedOdometry(imu, kinematics, config);
             }),
             py::arg("imu"),
             py::arg("kinematics"),
             py::arg("invert_x") = false,
             py::arg("invert_y") = false,
             py::arg("invert_z") = false,
             py::arg("invert_w") = false,
             py::keep_alive<1, 2>(),  // Keep IMU alive
             py::keep_alive<1, 3>(),  // Keep kinematics alive
             "Create fused odometry with IMU and kinematics model.\n\n"
             "Args:\n"
             "    imu: IMU sensor instance\n"
             "    kinematics: Kinematics model (provides velocity estimates)\n"
             "    invert_x: Invert quaternion x component (default: False)\n"
             "    invert_y: Invert quaternion y component (default: False)\n"
             "    invert_z: Invert quaternion z component (default: False)\n"
             "    invert_w: Invert quaternion w component (default: False)")
        .def("update", &libstp::odometry::fused::FusedOdometry::update,
             py::arg("dt"),
             "Update odometry estimate with time delta.\n\n"
             "Args:\n"
             "    dt: Time delta in seconds")
        .def("get_pose", &libstp::odometry::fused::FusedOdometry::getPose,
             "Get current pose estimate (position + orientation)")
        .def("reset", py::overload_cast<const libstp::foundation::Pose&>(&libstp::odometry::fused::FusedOdometry::reset),
             py::arg("pose"),
             "Reset odometry to a specific pose.\n\n"
             "Args:\n"
             "    pose: Pose to reset to")
        .def("reset", py::overload_cast<>(&libstp::odometry::fused::FusedOdometry::reset),
             "Reset odometry to origin (zero position, identity orientation)");
}
