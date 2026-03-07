#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/arc_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_arc_motion(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<ArcMotionConfig>(m, "ArcMotionConfig")
        .def(py::init<>())
        .def_readwrite("radius_m", &ArcMotionConfig::radius_m)
        .def_readwrite("arc_angle_rad", &ArcMotionConfig::arc_angle_rad)
        .def_readwrite("speed_scale", &ArcMotionConfig::speed_scale);

    py::class_<ArcMotionTelemetry>(m, "ArcMotionTelemetry")
        .def_readonly("time_s", &ArcMotionTelemetry::time_s)
        .def_readonly("dt", &ArcMotionTelemetry::dt)
        .def_readonly("target_angle_rad", &ArcMotionTelemetry::target_angle_rad)
        .def_readonly("heading_rad", &ArcMotionTelemetry::heading_rad)
        .def_readonly("heading_error_rad", &ArcMotionTelemetry::heading_error_rad)
        .def_readonly("arc_position_m", &ArcMotionTelemetry::arc_position_m)
        .def_readonly("arc_target_m", &ArcMotionTelemetry::arc_target_m)
        .def_readonly("filtered_velocity_radps", &ArcMotionTelemetry::filtered_velocity_radps)
        .def_readonly("cmd_vx_mps", &ArcMotionTelemetry::cmd_vx_mps)
        .def_readonly("cmd_wz_radps", &ArcMotionTelemetry::cmd_wz_radps)
        .def_readonly("pid_raw", &ArcMotionTelemetry::pid_raw)
        .def_readonly("setpoint_position_rad", &ArcMotionTelemetry::setpoint_position_rad)
        .def_readonly("setpoint_velocity_radps", &ArcMotionTelemetry::setpoint_velocity_radps)
        .def_readonly("saturated", &ArcMotionTelemetry::saturated);

    py::class_<ArcMotion, Motion, std::shared_ptr<ArcMotion>>(m, "ArcMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const ArcMotionConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<ArcMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &ArcMotion::start)
        .def("update", &ArcMotion::update, py::arg("dt"))
        .def("is_finished", &ArcMotion::isFinished)
        .def("get_telemetry", &ArcMotion::getTelemetry,
            py::return_value_policy::reference_internal);
}
