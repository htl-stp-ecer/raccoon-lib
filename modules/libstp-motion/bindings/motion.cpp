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
        .def(py::init([](py::kwargs kwargs) {
            UnifiedMotionPidConfig cfg;
            auto set_if = [&](const char* name, auto& field) {
                if (kwargs.contains(name)) {
                    field = kwargs[name].cast<std::decay_t<decltype(field)>>();
                }
            };

            // Distance PID gains
            set_if("distance_kp", cfg.distance_kp);
            set_if("distance_ki", cfg.distance_ki);
            set_if("distance_kd", cfg.distance_kd);
            // Heading PID gains
            set_if("heading_kp", cfg.heading_kp);
            set_if("heading_ki", cfg.heading_ki);
            set_if("heading_kd", cfg.heading_kd);
            // Lateral PID gains
            set_if("lateral_kp", cfg.lateral_kp);
            set_if("lateral_ki", cfg.lateral_ki);
            set_if("lateral_kd", cfg.lateral_kd);
            // Trapezoidal profile parameters
            // Advanced PID parameters
            set_if("integral_max", cfg.integral_max);
            set_if("integral_deadband", cfg.integral_deadband);
            set_if("derivative_lpf_alpha", cfg.derivative_lpf_alpha);
            set_if("output_min", cfg.output_min);
            set_if("output_max", cfg.output_max);
            // Saturation handling
            set_if("saturation_derating_factor", cfg.saturation_derating_factor);
            set_if("saturation_min_scale", cfg.saturation_min_scale);
            set_if("saturation_recovery_rate", cfg.saturation_recovery_rate);
            // Heading-specific saturation
            set_if("heading_saturation_derating_factor", cfg.heading_saturation_derating_factor);
            set_if("heading_min_scale", cfg.heading_min_scale);
            set_if("heading_recovery_rate", cfg.heading_recovery_rate);
            // Hysteresis parameters
            set_if("saturation_hold_cycles", cfg.saturation_hold_cycles);
            set_if("saturation_recovery_threshold", cfg.saturation_recovery_threshold);
            // Tolerances
            set_if("distance_tolerance_m", cfg.distance_tolerance_m);
            set_if("angle_tolerance_rad", cfg.angle_tolerance_rad);
            // Lateral drift handling
            set_if("lateral_heading_bias_gain", cfg.lateral_heading_bias_gain);
            set_if("lateral_reorient_threshold_m", cfg.lateral_reorient_threshold_m);
            set_if("heading_saturation_error_rad", cfg.heading_saturation_error_rad);
            set_if("heading_recovery_error_rad", cfg.heading_recovery_error_rad);
            // Minimum speeds
            set_if("min_speed_mps", cfg.min_speed_mps);
            // Response lag compensation
            set_if("response_lag_s", cfg.response_lag_s);

            return cfg;
        }))
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
        // Response lag compensation
        .def_readwrite("response_lag_s", &UnifiedMotionPidConfig::response_lag_s);

    py::class_<libstp::motion::Motion, std::shared_ptr<libstp::motion::Motion>>(m, "Motion")
        .def("start", &libstp::motion::Motion::start)
        .def("update", &libstp::motion::Motion::update, py::arg("dt"))
        .def("is_finished", &libstp::motion::Motion::isFinished);
}
