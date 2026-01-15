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
        .def_readwrite("max_speed_mps", &StrafeConfig::max_speed_mps);

    py::class_<StrafeMotion, Motion, std::shared_ptr<StrafeMotion>>(m, "StrafeMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         double distance_m,
                         double max_speed_mps)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<StrafeMotion>(ctx, distance_m, max_speed_mps);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("distance_m"),
            py::arg("max_speed_mps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const StrafeConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<StrafeMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &StrafeMotion::start)
        .def("update", &StrafeMotion::update, py::arg("dt"))
        .def("is_finished", &StrafeMotion::isFinished);
}
