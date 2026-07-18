#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/directional_line_follow_motion.hpp"
#include "motion/motion.hpp"

namespace py = pybind11;

void init_directional_line_follow_motion(py::module_& m)
{
    using namespace libstp::motion;

    py::enum_<LineFollowCorrectionMode>(m, "LineFollowCorrectionMode")
        .value("Angular", LineFollowCorrectionMode::Angular)
        .value("Lateral", LineFollowCorrectionMode::Lateral)
        .value("Forward", LineFollowCorrectionMode::Forward);

    py::class_<DirectionalLineFollowMotionConfig>(m, "DirectionalLineFollowMotionConfig")
        .def(py::init<>())
        .def_readwrite("heading_speed", &DirectionalLineFollowMotionConfig::heading_speed)
        .def_readwrite("strafe_speed", &DirectionalLineFollowMotionConfig::strafe_speed)
        .def_readwrite("distance_m", &DirectionalLineFollowMotionConfig::distance_m)
        .def_readwrite("has_distance_target", &DirectionalLineFollowMotionConfig::has_distance_target)
        .def_readwrite("kp", &DirectionalLineFollowMotionConfig::kp)
        .def_readwrite("ki", &DirectionalLineFollowMotionConfig::ki)
        .def_readwrite("kd", &DirectionalLineFollowMotionConfig::kd)
        .def_readwrite("integral_max", &DirectionalLineFollowMotionConfig::integral_max)
        .def_readwrite("integral_min", &DirectionalLineFollowMotionConfig::integral_min)
        .def_readwrite("correction_mode", &DirectionalLineFollowMotionConfig::correction_mode)
        .def_readwrite("heading_hold", &DirectionalLineFollowMotionConfig::heading_hold)
        .def_readwrite("correction_sign", &DirectionalLineFollowMotionConfig::correction_sign)
        .def_readwrite("has_target_heading", &DirectionalLineFollowMotionConfig::has_target_heading)
        .def_readwrite("target_heading_rad", &DirectionalLineFollowMotionConfig::target_heading_rad);

    py::class_<DirectionalLineFollowMotionTelemetry>(m, "DirectionalLineFollowMotionTelemetry")
        .def_readonly("time_s", &DirectionalLineFollowMotionTelemetry::time_s)
        .def_readonly("dt", &DirectionalLineFollowMotionTelemetry::dt)
        .def_readonly("error", &DirectionalLineFollowMotionTelemetry::error)
        .def_readonly("correction", &DirectionalLineFollowMotionTelemetry::correction)
        .def_readonly("heading_error_rad", &DirectionalLineFollowMotionTelemetry::heading_error_rad)
        .def_readonly("heading_speed_mps", &DirectionalLineFollowMotionTelemetry::heading_speed_mps)
        .def_readonly("strafe_speed_mps", &DirectionalLineFollowMotionTelemetry::strafe_speed_mps)
        .def_readonly("cmd_vx_mps", &DirectionalLineFollowMotionTelemetry::cmd_vx_mps)
        .def_readonly("cmd_vy_mps", &DirectionalLineFollowMotionTelemetry::cmd_vy_mps)
        .def_readonly("cmd_wz_radps", &DirectionalLineFollowMotionTelemetry::cmd_wz_radps)
        .def_readonly("straight_distance_m", &DirectionalLineFollowMotionTelemetry::straight_distance_m)
        .def_readonly("finished", &DirectionalLineFollowMotionTelemetry::finished);

    py::class_<DirectionalLineFollowMotion, Motion, std::shared_ptr<DirectionalLineFollowMotion>>(
        m,
        "DirectionalLineFollowMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const UnifiedMotionPidConfig& pid_config,
                         const DirectionalLineFollowMotionConfig& config)
        {
            MotionContext ctx{drive, odometry, pid_config};
            return std::make_shared<DirectionalLineFollowMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("pid_config"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            py::keep_alive<1, 4>())
        .def("start", &DirectionalLineFollowMotion::start)
        .def("update", &DirectionalLineFollowMotion::update, py::arg("dt"))
        .def("is_finished", &DirectionalLineFollowMotion::isFinished)
        .def("set_sensor_error", &DirectionalLineFollowMotion::setSensorError, py::arg("error"))
        .def("get_telemetry", &DirectionalLineFollowMotion::getTelemetry,
            py::return_value_policy::reference_internal);
}
