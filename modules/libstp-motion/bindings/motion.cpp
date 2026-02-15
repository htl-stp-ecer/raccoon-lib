#include <pybind11/pybind11.h>
#include "motion/motion.hpp"
#include "motion/motion_config.hpp"

namespace py = pybind11;

void init_motion_base(py::module_& m)
{
    using namespace libstp::motion;
    using namespace libstp::foundation;

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
    py::class_<UnifiedMotionPidConfig>(m, "UnifiedMotionPidConfig")
        .def(py::init([](
                 PidConfig distance, PidConfig heading,
                 double velocity_ff,
                 double saturation_derating_factor, double saturation_min_scale, double saturation_recovery_rate,
                 double heading_saturation_derating_factor, double heading_min_scale, double heading_recovery_rate,
                 int saturation_hold_cycles, double saturation_recovery_threshold,
                 double distance_tolerance_m, double angle_tolerance_rad,
                 double heading_saturation_error_rad, double heading_recovery_error_rad,
                 AxisConstraints linear, AxisConstraints lateral, AxisConstraints angular)
             {
                 UnifiedMotionPidConfig cfg;
                 cfg.distance = distance;
                 cfg.heading = heading;
                 cfg.velocity_ff = velocity_ff;
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
                 cfg.heading_saturation_error_rad = heading_saturation_error_rad;
                 cfg.heading_recovery_error_rad = heading_recovery_error_rad;
                 cfg.linear = linear;
                 cfg.lateral = lateral;
                 cfg.angular = angular;
                 return cfg;
             }),
             py::arg("distance") = PidConfig(2.0, 0.0, 0.5, 10.0, 0.01, 0.3),
             py::arg("heading") = PidConfig(2.0, 0.0, 0.3, 10.0, 0.01, 0.3),
             py::arg("velocity_ff") = 1.0,
             py::arg("saturation_derating_factor") = 0.9,
             py::arg("saturation_min_scale") = 0.2,
             py::arg("saturation_recovery_rate") = 0.03,
             py::arg("heading_saturation_derating_factor") = 0.85,
             py::arg("heading_min_scale") = 0.25,
             py::arg("heading_recovery_rate") = 0.05,
             py::arg("saturation_hold_cycles") = 5,
             py::arg("saturation_recovery_threshold") = 0.95,
             py::arg("distance_tolerance_m") = 0.01,
             py::arg("angle_tolerance_rad") = 0.035,
             py::arg("heading_saturation_error_rad") = 0.01,
             py::arg("heading_recovery_error_rad") = 0.005,
             py::arg("linear") = AxisConstraints{},
             py::arg("lateral") = AxisConstraints{},
             py::arg("angular") = AxisConstraints{}
        )
        .def_readwrite("distance", &UnifiedMotionPidConfig::distance)
        .def_readwrite("heading", &UnifiedMotionPidConfig::heading)
        .def_readwrite("velocity_ff", &UnifiedMotionPidConfig::velocity_ff)
        .def_readwrite("saturation_derating_factor", &UnifiedMotionPidConfig::saturation_derating_factor)
        .def_readwrite("saturation_min_scale", &UnifiedMotionPidConfig::saturation_min_scale)
        .def_readwrite("saturation_recovery_rate", &UnifiedMotionPidConfig::saturation_recovery_rate)
        .def_readwrite("heading_saturation_derating_factor", &UnifiedMotionPidConfig::heading_saturation_derating_factor)
        .def_readwrite("heading_min_scale", &UnifiedMotionPidConfig::heading_min_scale)
        .def_readwrite("heading_recovery_rate", &UnifiedMotionPidConfig::heading_recovery_rate)
        .def_readwrite("saturation_hold_cycles", &UnifiedMotionPidConfig::saturation_hold_cycles)
        .def_readwrite("saturation_recovery_threshold", &UnifiedMotionPidConfig::saturation_recovery_threshold)
        .def_readwrite("distance_tolerance_m", &UnifiedMotionPidConfig::distance_tolerance_m)
        .def_readwrite("angle_tolerance_rad", &UnifiedMotionPidConfig::angle_tolerance_rad)
        .def_readwrite("heading_saturation_error_rad", &UnifiedMotionPidConfig::heading_saturation_error_rad)
        .def_readwrite("heading_recovery_error_rad", &UnifiedMotionPidConfig::heading_recovery_error_rad)
        .def_readwrite("linear", &UnifiedMotionPidConfig::linear)
        .def_readwrite("lateral", &UnifiedMotionPidConfig::lateral)
        .def_readwrite("angular", &UnifiedMotionPidConfig::angular);

    py::class_<libstp::motion::Motion, std::shared_ptr<libstp::motion::Motion>>(m, "Motion")
        .def("start", &libstp::motion::Motion::start)
        .def("update", &libstp::motion::Motion::update, py::arg("dt"))
        .def("is_finished", &libstp::motion::Motion::isFinished);
}
