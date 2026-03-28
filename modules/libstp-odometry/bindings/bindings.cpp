//
// Created by tobias on 10/23/25.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "odometry/odometry.hpp"
#include "foundation/types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(odometry, m)
{
    m.doc() = "Python bindings for libstp-odometry";

    // Ensure shared foundation types are registered before exposing IOdometry.
    py::module_::import("libstp.foundation");

    py::class_<libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::IOdometry>>(m, "IOdometry")
        .def("update", &libstp::odometry::IOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::IOdometry::getPose)
        .def("get_absolute_heading", &libstp::odometry::IOdometry::getAbsoluteHeading,
             "Get absolute IMU heading unaffected by reset().\n\n"
             "Returns:\n"
             "    Heading in radians (CCW-positive). Stable across odometry resets.")
        .def("get_path_length", &libstp::odometry::IOdometry::getPathLength,
             "Get cumulative path length (odometer) in meters.\n\n"
             "Monotonically increasing — NOT affected by reset().\n"
             "Accumulates distance traveled each update cycle regardless of direction.\n\n"
             "Returns:\n"
             "    Total distance traveled in meters since construction.")
        .def("reset", py::overload_cast<>(&libstp::odometry::IOdometry::reset));
}
