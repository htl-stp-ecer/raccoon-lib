#include <pybind11/pybind11.h>
#include "motion/motion.hpp"
#include "motion/motion_config.hpp"

namespace py = pybind11;

void init_motion_base(py::module_& m)
{
    using namespace libstp::motion;

    // Unified Motion PID Configuration
    py::class_<UnifiedMotionPidConfig>(m, "UnifiedMotionPidConfig")
        .def(py::init<
                 double, double, double,
                 double, double, double,
                 double, double, double,
                 double, double, double, double, double,
                 double, double, double,
                 double, double, double,
                 int, double,
                 double, double,
                 double, double, double, double,
                 double,
                 double,
                 double,
                 double, double, double
             >(),
             // Distance PID gains
             py::arg("distance_kp") = 2.0,
             py::arg("distance_ki") = 0.0,
             py::arg("distance_kd") = 0.5,
             // Heading PID gains
             py::arg("heading_kp") = 2.0,
             py::arg("heading_ki") = 0.0,
             py::arg("heading_kd") = 0.3,
             // Lateral PID gains
             py::arg("lateral_kp") = 2.0,
             py::arg("lateral_ki") = 0.0,
             py::arg("lateral_kd") = 0.0,
             // Advanced PID parameters
             py::arg("integral_max") = 10.0,
             py::arg("integral_deadband") = 0.01,
             py::arg("derivative_lpf_alpha") = 0.3,
             py::arg("output_min") = -10.0,
             py::arg("output_max") = 10.0,
             // Saturation handling
             py::arg("saturation_derating_factor") = 0.9,
             py::arg("saturation_min_scale") = 0.2,
             py::arg("saturation_recovery_rate") = 0.03,
             // Heading-specific saturation
             py::arg("heading_saturation_derating_factor") = 0.85,
             py::arg("heading_min_scale") = 0.25,
             py::arg("heading_recovery_rate") = 0.05,
             // Hysteresis parameters
             py::arg("saturation_hold_cycles") = 5,
             py::arg("saturation_recovery_threshold") = 0.95,
             // Tolerances
             py::arg("distance_tolerance_m") = 0.01,
             py::arg("angle_tolerance_rad") = 0.035,
             // Lateral drift handling
             py::arg("lateral_heading_bias_gain") = 0.5,
             py::arg("lateral_reorient_threshold_m") = 0.15,
             py::arg("heading_saturation_error_rad") = 0.01,
             py::arg("heading_recovery_error_rad") = 0.005,
             // Minimum speeds
             py::arg("min_speed_mps") = 0.05,
             // Reorientation behavior
             py::arg("reorientation_speed_factor") = 0.3,
             // Response lag compensation
             py::arg("response_lag_s") = 0.3,
             // Braking-distance motion profile
             py::arg("decel_mps2") = 0.10,
             py::arg("rest_horizon_s") = 0.7,
             py::arg("horizon_blend_speed_mps") = 0.1
        )
        // Distance PID gains
        .def_readwrite("distance_kp", &UnifiedMotionPidConfig::distance_kp)
        .def_readwrite("distance_ki", &UnifiedMotionPidConfig::distance_ki)
        .def_readwrite("distance_kd", &UnifiedMotionPidConfig::distance_kd)
        // Heading PID gains
        .def_readwrite("heading_kp", &UnifiedMotionPidConfig::heading_kp)
        .def_readwrite("heading_ki", &UnifiedMotionPidConfig::heading_ki)
        .def_readwrite("heading_kd", &UnifiedMotionPidConfig::heading_kd)
        // Lateral PID gains
        .def_readwrite("lateral_kp", &UnifiedMotionPidConfig::lateral_kp)
        .def_readwrite("lateral_ki", &UnifiedMotionPidConfig::lateral_ki)
        .def_readwrite("lateral_kd", &UnifiedMotionPidConfig::lateral_kd)
        // Trapezoidal profile parameters
        // Advanced PID parameters
        .def_readwrite("integral_max", &UnifiedMotionPidConfig::integral_max)
        .def_readwrite("integral_deadband", &UnifiedMotionPidConfig::integral_deadband)
        .def_readwrite("derivative_lpf_alpha", &UnifiedMotionPidConfig::derivative_lpf_alpha)
        .def_readwrite("output_min", &UnifiedMotionPidConfig::output_min)
        .def_readwrite("output_max", &UnifiedMotionPidConfig::output_max)
        // Saturation handling
        .def_readwrite("saturation_derating_factor", &UnifiedMotionPidConfig::saturation_derating_factor)
        .def_readwrite("saturation_min_scale", &UnifiedMotionPidConfig::saturation_min_scale)
        .def_readwrite("saturation_recovery_rate", &UnifiedMotionPidConfig::saturation_recovery_rate)
        // Heading-specific saturation
        .def_readwrite("heading_saturation_derating_factor", &UnifiedMotionPidConfig::heading_saturation_derating_factor)
        .def_readwrite("heading_min_scale", &UnifiedMotionPidConfig::heading_min_scale)
        .def_readwrite("heading_recovery_rate", &UnifiedMotionPidConfig::heading_recovery_rate)
        // Hysteresis parameters
        .def_readwrite("saturation_hold_cycles", &UnifiedMotionPidConfig::saturation_hold_cycles)
        .def_readwrite("saturation_recovery_threshold", &UnifiedMotionPidConfig::saturation_recovery_threshold)
        // Tolerances
        .def_readwrite("distance_tolerance_m", &UnifiedMotionPidConfig::distance_tolerance_m)
        .def_readwrite("angle_tolerance_rad", &UnifiedMotionPidConfig::angle_tolerance_rad)
        // Lateral drift handling
        .def_readwrite("lateral_heading_bias_gain", &UnifiedMotionPidConfig::lateral_heading_bias_gain)
        .def_readwrite("lateral_reorient_threshold_m", &UnifiedMotionPidConfig::lateral_reorient_threshold_m)
        .def_readwrite("heading_saturation_error_rad", &UnifiedMotionPidConfig::heading_saturation_error_rad)
        .def_readwrite("heading_recovery_error_rad", &UnifiedMotionPidConfig::heading_recovery_error_rad)
        // Minimum speeds
        .def_readwrite("min_speed_mps", &UnifiedMotionPidConfig::min_speed_mps)
        // Reorientation behavior
        .def_readwrite("reorientation_speed_factor", &UnifiedMotionPidConfig::reorientation_speed_factor)
        // Response lag compensation
        .def_readwrite("response_lag_s", &UnifiedMotionPidConfig::response_lag_s)
        // Braking-distance motion profile
        .def_readwrite("decel_mps2", &UnifiedMotionPidConfig::decel_mps2)
        .def_readwrite("rest_horizon_s", &UnifiedMotionPidConfig::rest_horizon_s)
        .def_readwrite("horizon_blend_speed_mps", &UnifiedMotionPidConfig::horizon_blend_speed_mps);

    py::class_<libstp::motion::Motion, std::shared_ptr<libstp::motion::Motion>>(m, "Motion")
        .def("start", &libstp::motion::Motion::start)
        .def("update", &libstp::motion::Motion::update, py::arg("dt"))
        .def("is_finished", &libstp::motion::Motion::isFinished);
}
