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
        .def(py::init([](std::vector<int> gyro_orientation, std::vector<int> compass_orientation,
                         std::vector<int> axis_remap)
        {
            auto imu = std::make_shared<libstp::hal::imu::IMU>();
            auto& writer = platform::wombat::core::LcmDataWriter::instance();
            if (!gyro_orientation.empty())
            {
                if (gyro_orientation.size() != 9)
                    throw std::invalid_argument("gyro_orientation must have exactly 9 elements");
                int8_t m[9];
                for (int i = 0; i < 9; ++i) m[i] = static_cast<int8_t>(gyro_orientation[i]);
                writer.setImuGyroOrientation(m);
            }
            if (!compass_orientation.empty())
            {
                if (compass_orientation.size() != 9)
                    throw std::invalid_argument("compass_orientation must have exactly 9 elements");
                int8_t m[9];
                for (int i = 0; i < 9; ++i) m[i] = static_cast<int8_t>(compass_orientation[i]);
                writer.setImuCompassOrientation(m);
            }
            if (!axis_remap.empty())
            {
                if (axis_remap.size() != 9)
                    throw std::invalid_argument("axis_remap must have exactly 9 elements");
                int8_t m[9];
                for (int i = 0; i < 9; ++i) m[i] = static_cast<int8_t>(axis_remap[i]);
                writer.setAxisRemap(m);
            }
            return imu;
        }),
             py::arg("gyro_orientation") = std::vector<int>{},
             py::arg("compass_orientation") = std::vector<int>{},
             py::arg("axis_remap") = std::vector<int>{})
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
    .def("get_orientation", [](libstp::hal::imu::IMU& self)
    {
        const Eigen::Quaternionf orientation = self.getOrientation();
        return py::make_tuple(orientation.w(), orientation.x(), orientation.y(), orientation.z());
    }, "Get the current orientation as a quaternion (w, x, y, z)")
    .def_static("set_gyro_orientation", [](std::vector<int> matrix)
    {
        if (matrix.size() != 9)
            throw std::invalid_argument("Orientation matrix must have exactly 9 elements");
        int8_t m[9];
        for (int i = 0; i < 9; ++i) m[i] = static_cast<int8_t>(matrix[i]);
        platform::wombat::core::LcmDataWriter::instance().setImuGyroOrientation(m);
    }, py::arg("matrix"),
       "Set IMU gyro orientation matrix (9 int8 elements, 3x3 row-major)")
    .def_static("set_compass_orientation", [](std::vector<int> matrix)
    {
        if (matrix.size() != 9)
            throw std::invalid_argument("Orientation matrix must have exactly 9 elements");
        int8_t m[9];
        for (int i = 0; i < 9; ++i) m[i] = static_cast<int8_t>(matrix[i]);
        platform::wombat::core::LcmDataWriter::instance().setImuCompassOrientation(m);
    }, py::arg("matrix"),
       "Set IMU compass orientation matrix (9 int8 elements, 3x3 row-major)")
    .def_static("set_axis_remap", [](std::vector<int> matrix)
    {
        if (matrix.size() != 9)
            throw std::invalid_argument("Axis remap matrix must have exactly 9 elements");
        int8_t m[9];
        for (int i = 0; i < 9; ++i) m[i] = static_cast<int8_t>(matrix[i]);
        platform::wombat::core::LcmDataWriter::instance().setAxisRemap(m);
    }, py::arg("matrix"),
       "Set body-to-world axis remap matrix (9 int8 elements, 3x3 row-major). "
       "Applied in the data-reader to ALL sensor data before LCM publish.");
}
