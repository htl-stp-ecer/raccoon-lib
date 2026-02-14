#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/diagonal_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_diagonal_motion(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<DiagonalMotionConfig>(m, "DiagonalMotionConfig")
        .def(py::init<>())
        .def_readwrite("angle_rad", &DiagonalMotionConfig::angle_rad)
        .def_readwrite("distance_m", &DiagonalMotionConfig::distance_m)
        .def_readwrite("max_speed_mps", &DiagonalMotionConfig::max_speed_mps)
        .def_readwrite("max_acceleration_mps2", &DiagonalMotionConfig::max_acceleration_mps2)
        .def_readwrite("max_deceleration_mps2", &DiagonalMotionConfig::max_deceleration_mps2);

    py::class_<DiagonalMotionTelemetry>(m, "DiagonalMotionTelemetry")
        .def_readonly("time_s", &DiagonalMotionTelemetry::time_s)
        .def_readonly("dt", &DiagonalMotionTelemetry::dt)
        .def_readonly("target_m", &DiagonalMotionTelemetry::target_m)
        .def_readonly("position_m", &DiagonalMotionTelemetry::position_m)
        .def_readonly("predicted_m", &DiagonalMotionTelemetry::predicted_m)
        .def_readonly("cross_track_m", &DiagonalMotionTelemetry::cross_track_m)
        .def_readonly("distance_error_m", &DiagonalMotionTelemetry::distance_error_m)
        .def_readonly("actual_error_m", &DiagonalMotionTelemetry::actual_error_m)
        .def_readonly("yaw_error_rad", &DiagonalMotionTelemetry::yaw_error_rad)
        .def_readonly("filtered_velocity_mps", &DiagonalMotionTelemetry::filtered_velocity_mps)
        .def_readonly("cmd_vx_mps", &DiagonalMotionTelemetry::cmd_vx_mps)
        .def_readonly("cmd_vy_mps", &DiagonalMotionTelemetry::cmd_vy_mps)
        .def_readonly("cmd_wz_radps", &DiagonalMotionTelemetry::cmd_wz_radps)
        .def_readonly("pid_primary_raw", &DiagonalMotionTelemetry::pid_primary_raw)
        .def_readonly("pid_cross_raw", &DiagonalMotionTelemetry::pid_cross_raw)
        .def_readonly("pid_heading_raw", &DiagonalMotionTelemetry::pid_heading_raw)
        .def_readonly("setpoint_position_m", &DiagonalMotionTelemetry::setpoint_position_m)
        .def_readonly("setpoint_velocity_mps", &DiagonalMotionTelemetry::setpoint_velocity_mps)
        .def_readonly("heading_rad", &DiagonalMotionTelemetry::heading_rad)
        .def_readonly("speed_scale", &DiagonalMotionTelemetry::speed_scale)
        .def_readonly("heading_scale", &DiagonalMotionTelemetry::heading_scale)
        .def_readonly("saturated", &DiagonalMotionTelemetry::saturated);

    py::class_<DiagonalMotion, Motion, std::shared_ptr<DiagonalMotion>>(m, "DiagonalMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const DiagonalMotionConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<DiagonalMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &DiagonalMotion::start)
        .def("update", &DiagonalMotion::update, py::arg("dt"))
        .def("is_finished", &DiagonalMotion::isFinished)
        .def("get_telemetry", &DiagonalMotion::getTelemetry,
            py::return_value_policy::reference_internal);
}
