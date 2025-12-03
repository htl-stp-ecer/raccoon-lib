//
// Created by tobias on 27/11/25.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "calibration/motor/calibration.hpp"
#include "calibration/motion_calibration.hpp"
#include "foundation/motor.hpp"
#include "drive/drive.hpp"
#include "odometry/odometry.hpp"

namespace py = pybind11;

PYBIND11_MODULE(calibration, m)
{
    m.doc() = "Python bindings for motor calibration";

    // Ensure foundation types are available
    py::module_::import("libstp.foundation");

    // Calibration configuration
    py::class_<libstp::calibration::CalibrationConfig>(m, "CalibrationConfig")
        .def(py::init<>())
        .def_readwrite("step_response_amplitude", &libstp::calibration::CalibrationConfig::step_response_amplitude,
                      "Command percentage for step test (default: 30%)")
        .def_readwrite("step_response_duration", &libstp::calibration::CalibrationConfig::step_response_duration,
                      "Duration of step test in seconds (default: 3.0s)")
        .def_readwrite("relay_amplitude", &libstp::calibration::CalibrationConfig::relay_amplitude,
                      "Command percentage for relay test (default: 50%)")
        .def_readwrite("max_relay_duration", &libstp::calibration::CalibrationConfig::max_relay_duration,
                      "Maximum duration for relay test (default: 10.0s)")
        .def_readwrite("min_oscillations", &libstp::calibration::CalibrationConfig::min_oscillations,
                      "Minimum oscillations to measure (default: 3)")
        .def_readwrite("use_relay_feedback", &libstp::calibration::CalibrationConfig::use_relay_feedback,
                      "Use relay feedback (aggressive) instead of step response (conservative)")
        .def_readwrite("max_test_distance_m", &libstp::calibration::CalibrationConfig::max_test_distance_m,
                      "Emergency stop distance (default: 0.6m)")
        .def_readwrite("max_single_test_duration", &libstp::calibration::CalibrationConfig::max_single_test_duration,
                      "Timeout per individual test (default: 15.0s)")
        .def_readwrite("max_calibration_duration", &libstp::calibration::CalibrationConfig::max_calibration_duration,
                      "Total calibration timeout in seconds (default: 120s)")
        .def_readwrite("validation_duration", &libstp::calibration::CalibrationConfig::validation_duration,
                      "Duration of validation test (default: 2.0s)")
        .def_readwrite("validation_max_error", &libstp::calibration::CalibrationConfig::validation_max_error,
                      "Maximum acceptable tracking error (default: 0.2 = 20%)")
        .def("__repr__", [](const libstp::calibration::CalibrationConfig &c) {
            return "<CalibrationConfig use_relay=" + std::string(c.use_relay_feedback ? "True" : "False") + ">";
        });

    // Calibration result metrics
    py::class_<libstp::calibration::CalibrationResult::Metrics>(m, "CalibrationMetrics")
        .def(py::init<>())
        .def_readonly("static_friction_forward", &libstp::calibration::CalibrationResult::Metrics::static_friction_forward,
                     "Forward static friction threshold (%)")
        .def_readonly("static_friction_backward", &libstp::calibration::CalibrationResult::Metrics::static_friction_backward,
                     "Backward static friction threshold (%)")
        .def_readonly("velocity_constant_r_squared", &libstp::calibration::CalibrationResult::Metrics::velocity_constant_r_squared,
                     "Linear regression R² for velocity constant (0-1)")
        .def_readonly("velocity_samples", &libstp::calibration::CalibrationResult::Metrics::velocity_samples,
                     "Number of velocity test samples")
        .def_readonly("acceleration_mean", &libstp::calibration::CalibrationResult::Metrics::acceleration_mean,
                     "Mean acceleration constant")
        .def_readonly("acceleration_std_dev", &libstp::calibration::CalibrationResult::Metrics::acceleration_std_dev,
                     "Standard deviation of acceleration measurements")
        .def_readonly("time_constant_tau", &libstp::calibration::CalibrationResult::Metrics::time_constant_tau,
                     "System time constant (step response)")
        .def_readonly("steady_state_gain", &libstp::calibration::CalibrationResult::Metrics::steady_state_gain,
                     "System steady-state gain K (step response)")
        .def_readonly("delay", &libstp::calibration::CalibrationResult::Metrics::delay,
                     "System delay/dead time (step response)")
        .def_readonly("ultimate_gain_ku", &libstp::calibration::CalibrationResult::Metrics::ultimate_gain_ku,
                     "Ultimate gain Ku (relay feedback)")
        .def_readonly("ultimate_period_tu", &libstp::calibration::CalibrationResult::Metrics::ultimate_period_tu,
                     "Ultimate period Tu (relay feedback)")
        .def_readonly("oscillation_count", &libstp::calibration::CalibrationResult::Metrics::oscillation_count,
                     "Number of oscillations measured (relay feedback)")
        .def_readonly("validation_mean_error", &libstp::calibration::CalibrationResult::Metrics::validation_mean_error,
                     "Mean tracking error in validation test")
        .def_readonly("validation_max_error", &libstp::calibration::CalibrationResult::Metrics::validation_max_error,
                     "Maximum tracking error in validation test")
        .def_readonly("validation_passed", &libstp::calibration::CalibrationResult::Metrics::validation_passed,
                     "Whether validation passed");

    // Calibration result
    py::class_<libstp::calibration::CalibrationResult>(m, "CalibrationResult")
        .def(py::init<>())
        .def_readonly("pid", &libstp::calibration::CalibrationResult::pid,
                     "Tuned PID gains")
        .def_readonly("ff", &libstp::calibration::CalibrationResult::ff,
                     "Tuned feedforward parameters")
        .def_readonly("success", &libstp::calibration::CalibrationResult::success,
                     "Whether calibration succeeded")
        .def_readonly("error_message", &libstp::calibration::CalibrationResult::error_message,
                     "Error message if calibration failed")
        .def_readonly("duration_seconds", &libstp::calibration::CalibrationResult::duration_seconds,
                     "How long calibration took")
        .def_readonly("metrics", &libstp::calibration::CalibrationResult::metrics,
                     "Detailed calibration metrics")
        .def("__repr__", [](const libstp::calibration::CalibrationResult &r) {
            return std::string("<CalibrationResult success=") + (r.success ? "True" : "False") +
                   " kp=" + std::to_string(r.pid.kp) +
                   " ki=" + std::to_string(r.pid.ki) +
                   " kd=" + std::to_string(r.pid.kd) + ">";
        });

    // Motion calibration configuration
    py::class_<libstp::calibration::MotionCalibrationConfig>(m, "MotionCalibrationConfig")
        .def(py::init<>())
        .def_readwrite("initial_kp", &libstp::calibration::MotionCalibrationConfig::initial_kp,
                      "Initial proportional gain estimate")
        .def_readwrite("max_iterations", &libstp::calibration::MotionCalibrationConfig::max_iterations,
                      "Maximum tuning iterations per controller")
        .def_readwrite("target_settling_time", &libstp::calibration::MotionCalibrationConfig::target_settling_time,
                      "Target settling time in seconds")
        .def_readwrite("max_overshoot", &libstp::calibration::MotionCalibrationConfig::max_overshoot,
                      "Maximum allowed overshoot (as fraction, e.g. 0.15 = 15%)")
        .def_readwrite("target_steady_state_error", &libstp::calibration::MotionCalibrationConfig::target_steady_state_error,
                      "Target steady-state error")
        .def_readwrite("max_test_duration", &libstp::calibration::MotionCalibrationConfig::max_test_duration,
                      "Maximum total calibration duration in seconds")
        .def_readwrite("max_single_test_time", &libstp::calibration::MotionCalibrationConfig::max_single_test_time,
                      "Timeout for individual tests");

    // Motion type enum
    py::enum_<libstp::calibration::MotionType>(m, "MotionType")
        .value("TURN", libstp::calibration::MotionType::TURN)
        .value("DRIVE_STRAIGHT", libstp::calibration::MotionType::DRIVE_STRAIGHT)
        .value("STRAFE", libstp::calibration::MotionType::STRAFE);

    // Motion calibration gain set
    py::class_<libstp::calibration::MotionCalibrationResult::GainSet>(m, "MotionGainSet")
        .def(py::init<>())
        .def_readonly("kp", &libstp::calibration::MotionCalibrationResult::GainSet::kp)
        .def_readonly("ki", &libstp::calibration::MotionCalibrationResult::GainSet::ki)
        .def_readonly("kd", &libstp::calibration::MotionCalibrationResult::GainSet::kd)
        .def_readonly("motion_type", &libstp::calibration::MotionCalibrationResult::GainSet::motion_type)
        .def_readonly("controller_name", &libstp::calibration::MotionCalibrationResult::GainSet::controller_name)
        .def("__repr__", [](const libstp::calibration::MotionCalibrationResult::GainSet &g) {
            return "<MotionGainSet " + g.controller_name + " kp=" + std::to_string(g.kp) +
                   " ki=" + std::to_string(g.ki) + " kd=" + std::to_string(g.kd) + ">";
        });

    // Motion calibration result
    py::class_<libstp::calibration::MotionCalibrationResult>(m, "MotionCalibrationResult")
        .def(py::init<>())
        .def_readonly("gains", &libstp::calibration::MotionCalibrationResult::gains,
                     "List of calibrated gain sets")
        .def_readonly("success", &libstp::calibration::MotionCalibrationResult::success,
                     "Whether calibration succeeded")
        .def_readonly("error_message", &libstp::calibration::MotionCalibrationResult::error_message,
                     "Error message if failed")
        .def_readonly("duration_seconds", &libstp::calibration::MotionCalibrationResult::duration_seconds,
                     "Calibration duration")
        .def("__repr__", [](const libstp::calibration::MotionCalibrationResult &r) {
            return std::string("<MotionCalibrationResult success=") + (r.success ? "True" : "False") +
                   " gains=" + std::to_string(r.gains.size()) + ">";
        });

    // Motion calibrator
    py::class_<libstp::calibration::MotionCalibrator>(m, "MotionCalibrator")
        .def(py::init<libstp::drive::Drive&, libstp::odometry::IOdometry&>(),
             py::arg("drive"),
             py::arg("odometry"),
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>())
        .def(py::init<libstp::drive::Drive&, libstp::odometry::IOdometry&,
                      libstp::calibration::MotionCalibrationConfig>(),
             py::arg("drive"),
             py::arg("odometry"),
             py::arg("config"),
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>())
        .def("calibrate", &libstp::calibration::MotionCalibrator::calibrate,
             "Run full motion calibration (turn + drive straight + strafe if supported)")
        .def("calibrate_turn_motion", &libstp::calibration::MotionCalibrator::calibrateTurnMotion,
             "Calibrate only turn motion")
        .def("calibrate_drive_straight_motion", &libstp::calibration::MotionCalibrator::calibrateDriveStraightMotion,
             "Calibrate only drive straight motion")
        .def("calibrate_strafe_motion", &libstp::calibration::MotionCalibrator::calibrateStrafeMotion,
             "Calibrate only strafe motion");
}