//
// Python bindings for STM32-sourced odometry.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <memory>
#include <string>

#include "odometry/stm32/stm32_odometry.hpp"
#include "hal/IMU.hpp"
#include "hal/OdometryBridge.hpp"
#include "kinematics/kinematics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(odometry_stm32, m)
{
    m.doc() = "Python bindings for STM32-sourced odometry";

    py::module_::import("raccoon.odometry");
    py::module_::import("raccoon.hal");
    py::module_::import("raccoon.kinematics");

    // Stm32OdometryConfig struct
    py::class_<libstp::odometry::stm32::Stm32OdometryConfig>(m, "Stm32OdometryConfig",
            "Configuration for Stm32Odometry.")
        .def(py::init<int, std::string>(),
             py::arg("imu_ready_timeout_ms") = 1000,
             py::arg("turn_axis") = "world_z")
        .def_readwrite("imu_ready_timeout_ms",
                       &libstp::odometry::stm32::Stm32OdometryConfig::imu_ready_timeout_ms)
        .def_readwrite("turn_axis",
                       &libstp::odometry::stm32::Stm32OdometryConfig::turn_axis);

    py::class_<libstp::odometry::stm32::Stm32Odometry, libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::stm32::Stm32Odometry>>(
            m, "Stm32Odometry",
            "Odometry that reads dead-reckoning state from the STM32 coprocessor.\n"
            "The kinematics matrix is sent to the STM32 at construction time so it\n"
            "can run odometry on-board at full BEMF sample rate (~200Hz per motor).\n"
            "All getters return the latest snapshot from the coprocessor.")
        .def(py::init<std::shared_ptr<libstp::hal::imu::IMU>,
                      std::shared_ptr<libstp::kinematics::IKinematics>,
                      std::shared_ptr<libstp::hal::odometry_bridge::OdometryBridge>,
                      libstp::odometry::stm32::Stm32OdometryConfig>(),
             py::arg("imu"),
             py::arg("kinematics"),
             py::arg("bridge"),
             py::arg("config") = libstp::odometry::stm32::Stm32OdometryConfig{})
        .def("update", &libstp::odometry::stm32::Stm32Odometry::update, py::arg("dt"))
        .def("get_pose", &libstp::odometry::stm32::Stm32Odometry::getPose)
        .def("get_distance_from_origin", &libstp::odometry::stm32::Stm32Odometry::getDistanceFromOrigin)
        .def("get_heading", &libstp::odometry::stm32::Stm32Odometry::getHeading)
        .def("get_absolute_heading", &libstp::odometry::stm32::Stm32Odometry::getAbsoluteHeading)
        .def("get_path_length", &libstp::odometry::stm32::Stm32Odometry::getPathLength)
        .def("get_heading_error", &libstp::odometry::stm32::Stm32Odometry::getHeadingError,
             py::arg("target_heading_rad"))
        .def("reset", &libstp::odometry::stm32::Stm32Odometry::reset);
}
