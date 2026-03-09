//
// Created by tobias on 10/9/25.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>

#include "kinematics/differential/differential.hpp"
#include "calibration/motor/calibration.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kinematics_differential, m)
{
    m.doc() = "Python bindings for libstp-kinematics_differential";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("libstp.kinematics");
    py::module_::import("libstp.hal");
    py::module_::import("libstp.calibration");

    py::class_<libstp::kinematics::differential::DifferentialKinematics, libstp::kinematics::IKinematics,
                std::shared_ptr<libstp::kinematics::differential::DifferentialKinematics>>(
            m, "DifferentialKinematics")
        // Keep motor wrappers alive for at least as long as the kinematics object.
        .def(py::init<libstp::hal::motor::Motor*, libstp::hal::motor::Motor*, double, double>(),
             py::arg("left_motor"),
             py::arg("right_motor"),
             py::arg("wheelbase"),
             py::arg("wheel_radius"),
             py::keep_alive<1, 2>())
        .def("wheel_count", &libstp::kinematics::differential::DifferentialKinematics::wheelCount)
        .def("apply_command", &libstp::kinematics::differential::DifferentialKinematics::applyCommand,
             py::arg("cmd"), py::arg("dt"))
        .def("estimate_state", &libstp::kinematics::differential::DifferentialKinematics::estimateState)
        .def("hard_stop", &libstp::kinematics::differential::DifferentialKinematics::hardStop)
        .def("calibrate_motors",
             py::overload_cast<>(&libstp::kinematics::differential::DifferentialKinematics::calibrateMotors),
             "Calibrate both motors with default configuration")
        .def("calibrate_motors",
             py::overload_cast<const libstp::calibration::CalibrationConfig&>(
                 &libstp::kinematics::differential::DifferentialKinematics::calibrateMotors),
             py::arg("config"),
             "Calibrate both motors with custom configuration")
        .def("reset_encoders", &libstp::kinematics::differential::DifferentialKinematics::resetEncoders,
             "Reset encoder tracking to prevent stale deltas after odometry reset")
        .def("supports_lateral_motion", &libstp::kinematics::differential::DifferentialKinematics::supportsLateralMotion,
             "Returns False since differential drives cannot strafe")
        .def("get_wheel_radius", &libstp::kinematics::differential::DifferentialKinematics::getWheelRadius,
             "Get the wheel radius in meters")
        .def("apply_power_command", &libstp::kinematics::differential::DifferentialKinematics::applyPowerCommand,
             py::arg("direction"), py::arg("power_percent"),
             "Command motors at raw open-loop power using kinematics for direction")
        .def_property_readonly("motors", &libstp::kinematics::differential::DifferentialKinematics::getMotors,
             "List of motors managed by this kinematics model");
}
