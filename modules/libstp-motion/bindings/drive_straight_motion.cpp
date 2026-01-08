#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/drive_straight_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_drive_straight(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<DriveStraightConfig>(m, "DriveStraightConfig")
        .def(py::init<>())
        .def_readwrite("distance_m", &DriveStraightConfig::distance_m)
        .def_readwrite("max_speed_mps", &DriveStraightConfig::max_speed_mps);

    py::class_<DriveStraightMotion, Motion, std::shared_ptr<DriveStraightMotion>>(m, "DriveStraightMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         double distance_cm,
                         double max_speed_mps)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<DriveStraightMotion>(ctx, distance_cm, max_speed_mps);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("distance_cm"),
            py::arg("max_speed_mps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const DriveStraightConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<DriveStraightMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &DriveStraightMotion::start)
        .def("update", &DriveStraightMotion::update, py::arg("dt"))
        .def("is_finished", &DriveStraightMotion::isFinished);
}
