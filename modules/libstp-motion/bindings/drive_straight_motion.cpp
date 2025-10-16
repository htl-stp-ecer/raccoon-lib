#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/drive_straight_motion.hpp"

namespace py = pybind11;

void init_drive_straight(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<DriveStraightConfig>(m, "DriveStraightConfig")
        .def(py::init<>())
        .def_readwrite("distance_m", &DriveStraightConfig::distance_m)
        .def_readwrite("max_speed_mps", &DriveStraightConfig::max_speed_mps)
        .def_readwrite("distance_tolerance_m", &DriveStraightConfig::distance_tolerance_m)
        .def_readwrite("distance_kp", &DriveStraightConfig::distance_kp)
        .def_readwrite("heading_kp", &DriveStraightConfig::heading_kp)
        .def_readwrite("max_heading_rate", &DriveStraightConfig::max_heading_rate);

    py::class_<DriveStraightMotion, Motion, std::shared_ptr<DriveStraightMotion>>(m, "DriveStraightMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::hal::imu::IMU& imu,
                         double distance_cm,
                         double max_speed_mps)
        {
            MotionContext ctx{drive, imu};
            return std::make_shared<DriveStraightMotion>(ctx, distance_cm, max_speed_mps);
        }),
            py::arg("drive"),
            py::arg("imu"),
            py::arg("distance_cm"),
            py::arg("max_speed_mps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::hal::imu::IMU& imu,
                         const DriveStraightConfig& config)
        {
            MotionContext ctx{drive, imu};
            return std::make_shared<DriveStraightMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("imu"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def("start", &DriveStraightMotion::start)
        .def("update", &DriveStraightMotion::update, py::arg("dt"))
        .def("is_finished", &DriveStraightMotion::isFinished);
}
