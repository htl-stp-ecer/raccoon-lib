//
// Created by tobias on 5/19/25.
//
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <memory>
#include "hal/IMU.hpp"

namespace py = pybind11;

void init_imu(const py::module& m)
{
    py::class_<libstp::hal::imu::IMU, std::shared_ptr<libstp::hal::imu::IMU>>(m, "IMU")
        .def(py::init<>())
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
    .def("calibrate", &libstp::hal::imu::IMU::calibrate, "Calibrate the IMU sensor")
    .def("get_orientation", [](libstp::hal::imu::IMU& self)
    {
        const Eigen::Quaternionf orientation = self.getOrientation();
        return py::make_tuple(orientation.w(), orientation.x(), orientation.y(), orientation.z());
    }, "Get the current orientation as a quaternion (w, x, y, z)");
}
