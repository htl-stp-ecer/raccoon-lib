#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/turn_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_turn(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<TurnConfig>(m, "TurnConfig")
        .def(py::init<>())
        .def_readwrite("target_angle_rad", &TurnConfig::target_angle_rad)
        .def_readwrite("max_angular_rate", &TurnConfig::max_angular_rate);

    py::class_<TurnMotion, Motion, std::shared_ptr<TurnMotion>>(m, "TurnMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         double angle_deg,
                         double max_angular_rate_rad_per_sec)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<TurnMotion>(ctx, angle_deg, max_angular_rate_rad_per_sec);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("angle_deg"),
            py::arg("max_angular_rate_rad_per_sec"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const TurnConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<TurnMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &TurnMotion::start)
        .def("update", &TurnMotion::update, py::arg("dt"))
        .def("is_finished", &TurnMotion::isFinished);
}
