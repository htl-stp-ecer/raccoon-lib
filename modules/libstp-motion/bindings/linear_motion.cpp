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
        .def_readwrite("max_speed_mps", &LinearMotionConfig::max_speed_mps);

    py::class_<LinearMotionTelemetry>(m, "LinearMotionTelemetry")
        .def_readonly("time_s", &LinearMotionTelemetry::time_s)
        .def_readonly("dt", &LinearMotionTelemetry::dt)
        .def_readonly("target_m", &LinearMotionTelemetry::target_m)
        .def_readonly("position_m", &LinearMotionTelemetry::position_m)
        .def_readonly("predicted_m", &LinearMotionTelemetry::predicted_m)
        .def_readonly("cross_track_m", &LinearMotionTelemetry::cross_track_m)
        .def_readonly("distance_error_m", &LinearMotionTelemetry::distance_error_m)
        .def_readonly("actual_error_m", &LinearMotionTelemetry::actual_error_m)
        .def_readonly("yaw_error_rad", &LinearMotionTelemetry::yaw_error_rad)
        .def_readonly("filtered_velocity_mps", &LinearMotionTelemetry::filtered_velocity_mps)
        .def_readonly("cmd_vx_mps", &LinearMotionTelemetry::cmd_vx_mps)
        .def_readonly("cmd_vy_mps", &LinearMotionTelemetry::cmd_vy_mps)
        .def_readonly("cmd_wz_radps", &LinearMotionTelemetry::cmd_wz_radps)
        .def_readonly("pid_primary_raw", &LinearMotionTelemetry::pid_primary_raw)
        .def_readonly("pid_cross_raw", &LinearMotionTelemetry::pid_cross_raw)
        .def_readonly("pid_heading_raw", &LinearMotionTelemetry::pid_heading_raw)
        .def_readonly("heading_rad", &LinearMotionTelemetry::heading_rad)
        .def_readonly("speed_scale", &LinearMotionTelemetry::speed_scale)
        .def_readonly("heading_scale", &LinearMotionTelemetry::heading_scale)
        .def_readonly("saturated", &LinearMotionTelemetry::saturated);

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
        .def("is_finished", &LinearMotion::isFinished)
        .def("get_telemetry", &LinearMotion::getTelemetry,
            py::return_value_policy::reference_internal);
}
