#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/strafe_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_strafe(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<StrafeConfig>(m, "StrafeConfig")
        .def(py::init<>())
        .def_readwrite("target_distance_m", &StrafeConfig::target_distance_m)
        .def_readwrite("max_speed_mps", &StrafeConfig::max_speed_mps)
        .def_readwrite("distance_tolerance_m", &StrafeConfig::distance_tolerance_m)
        .def_readwrite("distance_kp", &StrafeConfig::distance_kp)
        .def_readwrite("distance_ki", &StrafeConfig::distance_ki)
        .def_readwrite("distance_kd", &StrafeConfig::distance_kd)
        .def_readwrite("heading_kp", &StrafeConfig::heading_kp)
        .def_readwrite("heading_ki", &StrafeConfig::heading_ki)
        .def_readwrite("heading_kd", &StrafeConfig::heading_kd)
        .def_readwrite("min_speed_mps", &StrafeConfig::min_speed_mps)
        .def_readwrite("saturation_derating_factor", &StrafeConfig::saturation_derating_factor)
        .def_readwrite("saturation_min_scale", &StrafeConfig::saturation_min_scale)
        .def_readwrite("saturation_recovery_rate", &StrafeConfig::saturation_recovery_rate);

    py::class_<StrafeMotion, Motion, std::shared_ptr<StrafeMotion>>(m, "StrafeMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         double distance_m,
                         double max_speed_mps)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<StrafeMotion>(ctx, distance_m, max_speed_mps);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("distance_m"),
            py::arg("max_speed_mps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const StrafeConfig& config)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<StrafeMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def("start", &StrafeMotion::start)
        .def("update", &StrafeMotion::update, py::arg("dt"))
        .def("is_finished", &StrafeMotion::isFinished);
}
