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

    // Ensure foundation types are registered
    py::module_::import("libstp.foundation");

    py::class_<libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::IOdometry>>(m, "IOdometry")
        .def("update", &libstp::odometry::IOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::IOdometry::getPose)
        .def("reset", py::overload_cast<const libstp::foundation::Pose&>(&libstp::odometry::IOdometry::reset),
             py::arg("pose"))
        .def("reset", py::overload_cast<>(&libstp::odometry::IOdometry::reset));
}
