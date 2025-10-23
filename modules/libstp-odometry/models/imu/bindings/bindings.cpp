//
// Created by tobias on 10/23/25.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "odometry/imu/imu_odometry.hpp"
#include "hal/IMU.hpp"

namespace py = pybind11;

PYBIND11_MODULE(odometry_imu, m)
{
    m.doc() = "Python bindings for libstp-odometry_imu";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("libstp.odometry");
    py::module_::import("libstp.hal");

    py::class_<libstp::odometry::imu::ImuOdometry, libstp::odometry::IOdometry>(
            m, "ImuOdometry")
        .def(py::init<libstp::hal::imu::IMU*>(),
             py::arg("imu"),
             py::keep_alive<1, 2>())
        .def("update", &libstp::odometry::imu::ImuOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::imu::ImuOdometry::getPose)
        .def("reset", py::overload_cast<const libstp::foundation::Pose&>(&libstp::odometry::imu::ImuOdometry::reset),
             py::arg("pose"))
        .def("reset", py::overload_cast<>(&libstp::odometry::imu::ImuOdometry::reset));
}
