#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/linear_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_linear_motion(py::module_& m)
{
    using namespace libstp::motion;

    py::enum_<LinearAxis>(m, "LinearAxis")
        .value("Forward", LinearAxis::Forward)
        .value("Lateral", LinearAxis::Lateral);

    py::class_<LinearMotionConfig>(m, "LinearMotionConfig")
        .def(py::init<>())
        .def_readwrite("axis", &LinearMotionConfig::axis)
        .def_readwrite("distance_m", &LinearMotionConfig::distance_m)
        .def_readwrite("max_speed_mps", &LinearMotionConfig::max_speed_mps)
        .def_readwrite("max_acceleration_mps2", &LinearMotionConfig::max_acceleration_mps2);

    py::class_<LinearMotion, Motion, std::shared_ptr<LinearMotion>>(m, "LinearMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const LinearMotionConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<LinearMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &LinearMotion::start)
        .def("update", &LinearMotion::update, py::arg("dt"))
        .def("is_finished", &LinearMotion::isFinished);
}
