//
// Created by tobias on 10/9/25.
//
#include <pybind11/pybind11.h>

#include "kinematics/differential/differential.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kinematics_differential, m)
{
    m.doc() = "Python bindings for libstp-kinematics_differential";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("libstp.kinematics");
    py::module_::import("libstp.hal");

    py::class_<libstp::kinematics::differential::DifferentialKinematics, libstp::kinematics::IKinematics>(
            m, "DifferentialKinematics")
        .def(py::init<libstp::hal::motor::Motor*, libstp::hal::motor::Motor*, double, double, double, double>(),
             py::arg("left_motor"),
             py::arg("right_motor"),
             py::arg("wheelbase"),
             py::arg("wheel_radius"),
             py::arg("max_velocity"),
             py::arg("max_acceleration"),
             py::keep_alive<1, 2>())
        .def("wheel_count", &libstp::kinematics::differential::DifferentialKinematics::wheelCount)
        .def("apply_command", &libstp::kinematics::differential::DifferentialKinematics::applyCommand,
             py::arg("cmd"), py::arg("dt"))
        .def("estimate_state", &libstp::kinematics::differential::DifferentialKinematics::estimateState)
        .def("hard_stop", &libstp::kinematics::differential::DifferentialKinematics::hardStop);
}
