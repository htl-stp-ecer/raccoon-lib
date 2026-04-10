//
// Created by tobias on 9/15/25.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>

#include "kinematics/kinematics.hpp"
#include "foundation/types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kinematics, m)
{
    m.doc() = "Python bindings for libstp-kinematics";

    // Import dependent bindings before exposing the abstract base interface.
    py::module_::import("raccoon.hal");

    py::class_<libstp::kinematics::IKinematics, std::shared_ptr<libstp::kinematics::IKinematics>>(m, "IKinematics")
        .def("wheel_count", &libstp::kinematics::IKinematics::wheelCount)
        .def("apply_command", &libstp::kinematics::IKinematics::applyCommand,
             py::arg("cmd"), py::arg("dt"))
        .def("estimate_state", &libstp::kinematics::IKinematics::estimateState)
        .def("hard_stop", &libstp::kinematics::IKinematics::hardStop)
        .def("apply_power_command", &libstp::kinematics::IKinematics::applyPowerCommand,
             py::arg("direction"), py::arg("power_percent"),
             "Command motors at raw open-loop power using kinematics for direction")
        .def("supports_lateral_motion", &libstp::kinematics::IKinematics::supportsLateralMotion,
             "Query whether the drivetrain can strafe sideways (vy motion)")
        .def_property_readonly("motors", &libstp::kinematics::IKinematics::getMotors,
             "List of motors managed by this kinematics model");
}
