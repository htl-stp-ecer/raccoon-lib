#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/spline_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_spline_motion(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<SplineMotionConfig>(m, "SplineMotionConfig")
        .def(py::init<>())
        .def_readwrite("waypoints_m", &SplineMotionConfig::waypoints_m)
        .def_readwrite("headings_rad", &SplineMotionConfig::headings_rad)
        .def_readwrite("speed_scale", &SplineMotionConfig::speed_scale);

    py::class_<SplineMotionTelemetry>(m, "SplineMotionTelemetry")
        .def_readonly("time_s", &SplineMotionTelemetry::time_s)
        .def_readonly("dt", &SplineMotionTelemetry::dt)
        .def_readonly("arc_length_m", &SplineMotionTelemetry::arc_length_m)
        .def_readonly("arc_target_m", &SplineMotionTelemetry::arc_target_m)
        .def_readonly("cross_track_m", &SplineMotionTelemetry::cross_track_m)
        .def_readonly("heading_rad", &SplineMotionTelemetry::heading_rad)
        .def_readonly("target_heading_rad", &SplineMotionTelemetry::target_heading_rad)
        .def_readonly("heading_error_rad", &SplineMotionTelemetry::heading_error_rad)
        .def_readonly("filtered_velocity_mps", &SplineMotionTelemetry::filtered_velocity_mps)
        .def_readonly("cmd_vx_mps", &SplineMotionTelemetry::cmd_vx_mps)
        .def_readonly("cmd_vy_mps", &SplineMotionTelemetry::cmd_vy_mps)
        .def_readonly("cmd_wz_radps", &SplineMotionTelemetry::cmd_wz_radps)
        .def_readonly("setpoint_position_m", &SplineMotionTelemetry::setpoint_position_m)
        .def_readonly("setpoint_velocity_mps", &SplineMotionTelemetry::setpoint_velocity_mps)
        .def_readonly("saturated", &SplineMotionTelemetry::saturated);

    py::class_<SplineMotion, Motion, std::shared_ptr<SplineMotion>>(m, "SplineMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const SplineMotionConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<SplineMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &SplineMotion::start)
        .def("update", &SplineMotion::update, py::arg("dt"))
        .def("is_finished", &SplineMotion::isFinished)
        .def("get_telemetry", &SplineMotion::getTelemetry,
            py::return_value_policy::reference_internal);
}
