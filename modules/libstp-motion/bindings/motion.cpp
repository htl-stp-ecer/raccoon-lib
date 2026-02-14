#include <pybind11/pybind11.h>
#include "motion/motion.hpp"
#include "motion/motion_config.hpp"

namespace py = pybind11;

void init_motion_base(py::module_& m)
{
    using namespace libstp::motion;

    // Per-axis motion profile constraints
    py::class_<AxisConstraints>(m, "AxisConstraints")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("max_velocity") = 0.0,
             py::arg("acceleration") = 0.0,
             py::arg("deceleration") = 0.0)
        .def_readwrite("max_velocity", &AxisConstraints::max_velocity)
        .def_readwrite("acceleration", &AxisConstraints::acceleration)
        .def_readwrite("deceleration", &AxisConstraints::deceleration);

    // Unified Motion PID Configuration
    // The lambda constructor accepts flat kwarg names (matching the YAML code generator's
    // flatten convention) and maps axis constraint params to the nested AxisConstraints.
    py::class_<UnifiedMotionPidConfig>(m, "UnifiedMotionPidConfig")
        .def(py::init([](
                 double distance_kp, double distance_ki, double distance_kd,
                 double heading_kp, double heading_ki, double heading_kd,
                 double lateral_kp, double lateral_ki, double lateral_kd,
                 double velocity_ff,
                 double integral_max, double integral_deadband, double derivative_lpf_alpha,
                 double output_min, double output_max,
                 double saturation_derating_factor, double saturation_min_scale, double saturation_recovery_rate,
                 double heading_saturation_derating_factor, double heading_min_scale, double heading_recovery_rate,
                 int saturation_hold_cycles, double saturation_recovery_threshold,
                 double distance_tolerance_m, double angle_tolerance_rad,
                 double lateral_heading_bias_gain, double lateral_reorient_threshold_m,
                 double heading_saturation_error_rad, double heading_recovery_error_rad,
                 double reorientation_speed_factor,
                 double default_linear_acceleration_mps2, double default_linear_deceleration_mps2,
                 double default_linear_max_velocity_mps,
                 double default_lateral_acceleration_mps2, double default_lateral_deceleration_mps2,
                 double default_lateral_max_velocity_mps,
                 double default_angular_acceleration_radps2, double default_angular_deceleration_radps2,
                 double default_angular_max_rate_radps)
             {
                 UnifiedMotionPidConfig cfg;
                 cfg.distance_kp = distance_kp;
                 cfg.distance_ki = distance_ki;
                 cfg.distance_kd = distance_kd;
                 cfg.heading_kp = heading_kp;
                 cfg.heading_ki = heading_ki;
                 cfg.heading_kd = heading_kd;
                 cfg.lateral_kp = lateral_kp;
                 cfg.lateral_ki = lateral_ki;
                 cfg.lateral_kd = lateral_kd;
                 cfg.velocity_ff = velocity_ff;
                 cfg.integral_max = integral_max;
                 cfg.integral_deadband = integral_deadband;
                 cfg.derivative_lpf_alpha = derivative_lpf_alpha;
                 cfg.output_min = output_min;
                 cfg.output_max = output_max;
                 cfg.saturation_derating_factor = saturation_derating_factor;
                 cfg.saturation_min_scale = saturation_min_scale;
                 cfg.saturation_recovery_rate = saturation_recovery_rate;
                 cfg.heading_saturation_derating_factor = heading_saturation_derating_factor;
                 cfg.heading_min_scale = heading_min_scale;
                 cfg.heading_recovery_rate = heading_recovery_rate;
                 cfg.saturation_hold_cycles = saturation_hold_cycles;
                 cfg.saturation_recovery_threshold = saturation_recovery_threshold;
                 cfg.distance_tolerance_m = distance_tolerance_m;
                 cfg.angle_tolerance_rad = angle_tolerance_rad;
                 cfg.lateral_heading_bias_gain = lateral_heading_bias_gain;
                 cfg.lateral_reorient_threshold_m = lateral_reorient_threshold_m;
                 cfg.heading_saturation_error_rad = heading_saturation_error_rad;
                 cfg.heading_recovery_error_rad = heading_recovery_error_rad;
                 cfg.reorientation_speed_factor = reorientation_speed_factor;
                 cfg.linear = {default_linear_max_velocity_mps, default_linear_acceleration_mps2, default_linear_deceleration_mps2};
                 cfg.lateral = {default_lateral_max_velocity_mps, default_lateral_acceleration_mps2, default_lateral_deceleration_mps2};
                 cfg.angular = {default_angular_max_rate_radps, default_angular_acceleration_radps2, default_angular_deceleration_radps2};
                 return cfg;
             }),
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
             // Profiled PID velocity feedforward
             py::arg("velocity_ff") = 1.0,
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
             // Reorientation behavior
             py::arg("reorientation_speed_factor") = 0.3,
             // Forward axis profile constraints
             py::arg("default_linear_acceleration_mps2") = 0.25,
             py::arg("default_linear_deceleration_mps2") = 0.037,
             py::arg("default_linear_max_velocity_mps") = 0.0,
             // Lateral axis profile constraints
             py::arg("default_lateral_acceleration_mps2") = 0.0,
             py::arg("default_lateral_deceleration_mps2") = 0.0,
             py::arg("default_lateral_max_velocity_mps") = 0.0,
             // Angular axis profile constraints
             py::arg("default_angular_acceleration_radps2") = 0.0,
             py::arg("default_angular_deceleration_radps2") = 0.0,
             py::arg("default_angular_max_rate_radps") = 0.0
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
        // Profiled PID velocity feedforward
        .def_readwrite("velocity_ff", &UnifiedMotionPidConfig::velocity_ff)
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
        // Reorientation behavior
        .def_readwrite("reorientation_speed_factor", &UnifiedMotionPidConfig::reorientation_speed_factor)
        // Per-axis profile constraints
        .def_readwrite("linear", &UnifiedMotionPidConfig::linear)
        .def_readwrite("lateral", &UnifiedMotionPidConfig::lateral)
        .def_readwrite("angular", &UnifiedMotionPidConfig::angular);

    py::class_<libstp::motion::Motion, std::shared_ptr<libstp::motion::Motion>>(m, "Motion")
        .def("start", &libstp::motion::Motion::start)
        .def("update", &libstp::motion::Motion::update, py::arg("dt"))
        .def("is_finished", &libstp::motion::Motion::isFinished);
}
