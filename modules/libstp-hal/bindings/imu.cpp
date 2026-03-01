//
// Created by tobias on 5/19/25.
//
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <memory>
#include "hal/IMU.hpp"
#include "core/LcmWriter.hpp"

namespace py = pybind11;

void init_imu(const py::module& m)
{
    py::class_<libstp::hal::imu::IMU, std::shared_ptr<libstp::hal::imu::IMU>>(m, "IMU")
        .def(py::init([]()
        {
            auto imu = std::make_shared<libstp::hal::imu::IMU>();
            return imu;
        }), "Create an IMU instance")
        .def("read", [](libstp::hal::imu::IMU& self)
        {
            float accel[3] = {0.0f, 0.0f, 0.0f};
            float gyro[3] = {0.0f, 0.0f, 0.0f};
            float magneto[3] = {0.0f, 0.0f, 0.0f};

            self.read(accel, gyro, magneto);

            py::tuple result(3);
            result[0] = py::make_tuple(accel[0], accel[1], accel[2]);
            result[1] = py::make_tuple(gyro[0], gyro[1], gyro[2]);
            result[2] = py::make_tuple(magneto[0], magneto[1], magneto[2]);

            return result;
        }, "Read acceleration, gyroscope, and magnetometer data")
    .def("get_angular_velocity", [](libstp::hal::imu::IMU& self)
    {
        float gyro[3] = {0.0f, 0.0f, 0.0f};
        self.getAngularVelocity(gyro);
        return py::make_tuple(gyro[0], gyro[1], gyro[2]);
    }, "Get angular velocity (gyroscope) as (x, y, z) in rad/s")
    .def("get_yaw_rate", &libstp::hal::imu::IMU::getYawRate,
         "Get angular rate along the configured turn axis in rad/s "
         "(default mode: world_z)")
    .def("set_yaw_rate_axis_mode",
         py::overload_cast<const std::string&>(&libstp::hal::imu::IMU::setYawRateAxisMode),
         py::arg("mode"),
         "Set turn-rate axis mode used by get_yaw_rate(): "
         "'world_z' (default), 'body_x', 'body_y', or 'body_z'")
    .def("get_yaw_rate_axis_mode", &libstp::hal::imu::IMU::getYawRateAxisModeName,
         "Get current turn-rate axis mode string")
    .def("calibrate", &libstp::hal::imu::IMU::calibrate, "Calibrate the IMU sensor")
    .def("get_heading", &libstp::hal::imu::IMU::getHeading,
         "Get firmware-computed heading in radians");
}
