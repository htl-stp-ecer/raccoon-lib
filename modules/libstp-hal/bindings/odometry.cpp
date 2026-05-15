#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "hal/odometry.hpp"
#include "foundation/types.hpp"

namespace py = pybind11;

void init_odometry(py::module& m)
{
    py::class_<libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::IOdometry>>(m, "IOdometry")
        .def("update", &libstp::odometry::IOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::IOdometry::getPose)
        .def("get_absolute_heading", &libstp::odometry::IOdometry::getAbsoluteHeading)
        .def("get_path_length", &libstp::odometry::IOdometry::getPathLength)
        .def("reset", py::overload_cast<>(&libstp::odometry::IOdometry::reset));
}
