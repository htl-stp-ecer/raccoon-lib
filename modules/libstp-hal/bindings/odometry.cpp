#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "hal/odometry.hpp"
#include "foundation/types.hpp"

namespace py = pybind11;

void init_odometry(py::module& m)
{
    py::class_<libstp::odometry::DistanceFromOrigin>(m, "DistanceFromOrigin")
        .def_readonly("forward", &libstp::odometry::DistanceFromOrigin::forward)
        .def_readonly("lateral", &libstp::odometry::DistanceFromOrigin::lateral)
        .def_readonly("straight_line", &libstp::odometry::DistanceFromOrigin::straight_line)
        .def("__repr__", [](const libstp::odometry::DistanceFromOrigin& d) {
            return "DistanceFromOrigin(forward=" + std::to_string(d.forward)
                + ", lateral=" + std::to_string(d.lateral)
                + ", straight_line=" + std::to_string(d.straight_line) + ")";
        });

    py::class_<libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::IOdometry>>(m, "IOdometry")
        .def("update", &libstp::odometry::IOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::IOdometry::getPose)
        .def("get_distance_from_origin", &libstp::odometry::IOdometry::getDistanceFromOrigin)
        .def("get_heading", &libstp::odometry::IOdometry::getHeading)
        .def("get_heading_error", &libstp::odometry::IOdometry::getHeadingError,
             py::arg("target_heading_rad"))
        .def("get_absolute_heading", &libstp::odometry::IOdometry::getAbsoluteHeading)
        .def("get_path_length", &libstp::odometry::IOdometry::getPathLength)
        .def("reset", py::overload_cast<>(&libstp::odometry::IOdometry::reset));
}
