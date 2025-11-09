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

    // Bind configuration struct
    py::class_<libstp::odometry::imu::ImuOdometryConfig>(m, "ImuOdometryConfig")
        .def(py::init<>())
        .def_readwrite("invert_x", &libstp::odometry::imu::ImuOdometryConfig::invert_x,
                       "Invert quaternion x component")
        .def_readwrite("invert_y", &libstp::odometry::imu::ImuOdometryConfig::invert_y,
                       "Invert quaternion y component")
        .def_readwrite("invert_z", &libstp::odometry::imu::ImuOdometryConfig::invert_z,
                       "Invert quaternion z component")
        .def_readwrite("invert_w", &libstp::odometry::imu::ImuOdometryConfig::invert_w,
                       "Invert quaternion w component (for 180° rotation)");

    py::class_<libstp::odometry::imu::ImuOdometry, libstp::odometry::IOdometry>(
            m, "ImuOdometry")
        .def(py::init([](libstp::hal::imu::IMU* imu,
                         bool invert_x,
                         bool invert_y,
                         bool invert_z,
                         bool invert_w) {
                 libstp::odometry::imu::ImuOdometryConfig config{};
                 config.invert_x = invert_x;
                 config.invert_y = invert_y;
                 config.invert_z = invert_z;
                 config.invert_w = invert_w;
                 return new libstp::odometry::imu::ImuOdometry(imu, config);
             }),
             py::arg("imu"),
             py::arg("invert_x") = false,
             py::arg("invert_y") = false,
             py::arg("invert_z") = false,
             py::arg("invert_w") = false,
             py::keep_alive<1, 2>())
        .def("update", &libstp::odometry::imu::ImuOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::imu::ImuOdometry::getPose)
        .def("reset", py::overload_cast<const libstp::foundation::Pose&>(&libstp::odometry::imu::ImuOdometry::reset),
             py::arg("pose"))
        .def("reset", py::overload_cast<>(&libstp::odometry::imu::ImuOdometry::reset));
}
