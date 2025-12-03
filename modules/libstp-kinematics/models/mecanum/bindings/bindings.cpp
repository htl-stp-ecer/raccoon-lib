//
// Created by tobias on 10/9/25.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "kinematics/mecanum/mecanum.hpp"
#include "calibration/motor/calibration.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kinematics_mecanum, m)
{
    m.doc() = "Python bindings for libstp-kinematics_mecanum";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("libstp.kinematics");
    py::module_::import("libstp.hal");
    py::module_::import("libstp.calibration");

    py::class_<libstp::kinematics::mecanum::MecanumKinematics, libstp::kinematics::IKinematics>(m, "MecanumKinematics")
        .def(py::init<libstp::hal::motor::Motor*,
                      libstp::hal::motor::Motor*,
                      libstp::hal::motor::Motor*,
                      libstp::hal::motor::Motor*,
                      double, double, double, double, double>(),
             py::arg("front_left_motor"),
             py::arg("front_right_motor"),
             py::arg("back_left_motor"),
             py::arg("back_right_motor"),
             py::arg("wheelbase"),
             py::arg("track_width"),
             py::arg("wheel_radius"),
             py::arg("max_velocity"),
             py::arg("max_acceleration"),
             py::keep_alive<1, 2>())
        .def("wheel_count", &libstp::kinematics::mecanum::MecanumKinematics::wheelCount)
        .def("apply_command", &libstp::kinematics::mecanum::MecanumKinematics::applyCommand,
             py::arg("cmd"), py::arg("dt"))
        .def("estimate_state", &libstp::kinematics::mecanum::MecanumKinematics::estimateState)
        .def("hard_stop", &libstp::kinematics::mecanum::MecanumKinematics::hardStop)
        .def("calibrate_motors",
             py::overload_cast<>(&libstp::kinematics::mecanum::MecanumKinematics::calibrateMotors),
             "Calibrate all 4 motors with default configuration")
        .def("calibrate_motors",
             py::overload_cast<const libstp::calibration::CalibrationConfig&>(
                 &libstp::kinematics::mecanum::MecanumKinematics::calibrateMotors),
             py::arg("config"),
             "Calibrate all 4 motors with custom configuration");
}
