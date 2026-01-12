#include <pybind11/pybind11.h>
#include "motion/motion.hpp"
#include "motion/motion_config.hpp"

namespace py = pybind11;

void init_motion_base(py::module_& m)
{
    using namespace libstp::motion;

    // Unified Motion PID Configuration
    py::class_<UnifiedMotionPidConfig>(m, "UnifiedMotionPidConfig")
        .def(py::init<>())
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
        .def_readwrite("max_linear_acceleration", &UnifiedMotionPidConfig::max_linear_acceleration)
        .def_readwrite("max_angular_acceleration", &UnifiedMotionPidConfig::max_angular_acceleration)
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
        // Tolerances
        .def_readwrite("distance_tolerance_m", &UnifiedMotionPidConfig::distance_tolerance_m)
        .def_readwrite("angle_tolerance_rad", &UnifiedMotionPidConfig::angle_tolerance_rad)
        // Rate limits
        .def_readwrite("max_heading_rate", &UnifiedMotionPidConfig::max_heading_rate)
        .def_readwrite("min_angular_rate", &UnifiedMotionPidConfig::min_angular_rate)
        // Lateral drift handling
        .def_readwrite("lateral_heading_bias_gain", &UnifiedMotionPidConfig::lateral_heading_bias_gain)
        .def_readwrite("lateral_reorient_threshold_m", &UnifiedMotionPidConfig::lateral_reorient_threshold_m)
        .def_readwrite("heading_saturation_error_rad", &UnifiedMotionPidConfig::heading_saturation_error_rad)
        .def_readwrite("heading_recovery_error_rad", &UnifiedMotionPidConfig::heading_recovery_error_rad)
        // Minimum speeds
        .def_readwrite("min_speed_mps", &UnifiedMotionPidConfig::min_speed_mps);

    py::class_<libstp::motion::Motion, std::shared_ptr<libstp::motion::Motion>>(m, "Motion")
        .def("start", &libstp::motion::Motion::start)
        .def("update", &libstp::motion::Motion::update, py::arg("dt"))
        .def("is_finished", &libstp::motion::Motion::isFinished);
}
