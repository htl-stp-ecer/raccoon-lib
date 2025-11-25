#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/turn_motion.hpp"

namespace py = pybind11;

void init_turn(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<TurnConfig>(m, "TurnConfig")
        .def(py::init<>())
        .def_readwrite("target_angle_rad", &TurnConfig::target_angle_rad)
        .def_readwrite("max_angular_rate", &TurnConfig::max_angular_rate)
        .def_readwrite("angle_tolerance_rad", &TurnConfig::angle_tolerance_rad)
        .def_readwrite("angle_kp", &TurnConfig::angle_kp)
        .def_readwrite("min_angular_rate", &TurnConfig::min_angular_rate)
        .def_readwrite("saturation_derating_factor", &TurnConfig::saturation_derating_factor)
        .def_readwrite("saturation_min_scale", &TurnConfig::saturation_min_scale)
        .def_readwrite("saturation_recovery_rate", &TurnConfig::saturation_recovery_rate);

    py::class_<TurnMotion, Motion, std::shared_ptr<TurnMotion>>(m, "TurnMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         double angle_deg,
                         double max_angular_rate_rad_per_sec)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<TurnMotion>(ctx, angle_deg, max_angular_rate_rad_per_sec);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("angle_deg"),
            py::arg("max_angular_rate_rad_per_sec"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const TurnConfig& config)
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
