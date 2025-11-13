#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/turn_motion.hpp"

namespace py = pybind11;

void init_turn_motion(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<TurnMotionConfig>(m, "TurnMotionConfig")
        .def(py::init<>())
        .def_readwrite("angle_deg", &TurnMotionConfig::angle_deg)
        .def_readwrite("max_angular_speed_rps", &TurnMotionConfig::max_angular_speed_rps)
        .def_readwrite("angle_tolerance_deg", &TurnMotionConfig::angle_tolerance_deg)
        .def_readwrite("angle_kp", &TurnMotionConfig::angle_kp)
        .def_readwrite("saturation_derating_factor", &TurnMotionConfig::saturation_derating_factor)
        .def_readwrite("saturation_min_speed_scale", &TurnMotionConfig::saturation_min_speed_scale)
        .def_readwrite("saturation_recovery_rate", &TurnMotionConfig::saturation_recovery_rate);

    py::class_<TurnMotion, Motion, std::shared_ptr<TurnMotion>>(m, "TurnMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         double angle_deg,
                         double max_angular_speed_rps)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<TurnMotion>(ctx, angle_deg, max_angular_speed_rps);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("angle_deg"),
            py::arg("max_angular_speed_rps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const TurnMotionConfig& config)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<TurnMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def("start", &TurnMotion::start)
        .def("update", &TurnMotion::update, py::arg("dt"))
        .def("is_finished", &TurnMotion::isFinished);
}
