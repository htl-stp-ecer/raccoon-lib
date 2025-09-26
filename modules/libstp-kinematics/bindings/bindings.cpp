//
// Created by tobias on 9/15/25.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "kinematics/kinematics.hpp"
#include "kinematics/differential/differential.hpp"
#include "kinematics/mecanum/mecanum.hpp"
#include "foundation/types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kinematics, m) {
    m.doc() = "Python bindings for libstp-kinematics";

    // Foundation types
    py::class_<libstp::foundation::ChassisCmd>(m, "ChassisCmd")
        .def(py::init<double, double, double>())
        .def_readwrite("vx", &libstp::foundation::ChassisCmd::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisCmd::vy)
        .def_readwrite("wz", &libstp::foundation::ChassisCmd::wz);

    py::class_<libstp::foundation::ChassisState>(m, "ChassisState")
        .def(py::init<double, double, double>())
        .def_readwrite("vx", &libstp::foundation::ChassisState::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisState::vy)
        .def_readwrite("wz", &libstp::foundation::ChassisState::wz);

    // Base kinematics interface
    py::class_<libstp::kinematics::IKinematics>(m, "IKinematics")
        .def("wheelCount", &libstp::kinematics::IKinematics::wheelCount)
        .def("inverse", &libstp::kinematics::IKinematics::inverse)
        .def("forward", &libstp::kinematics::IKinematics::forward);

    // Differential kinematics
    py::class_<libstp::kinematics::differential::DifferentialKinematics, libstp::kinematics::IKinematics>(m, "DifferentialKinematics")
        .def(py::init<double, double>())
        .def("wheelCount", &libstp::kinematics::differential::DifferentialKinematics::wheelCount)
        .def("inverse", &libstp::kinematics::differential::DifferentialKinematics::inverse)
        .def("forward", &libstp::kinematics::differential::DifferentialKinematics::forward);

    // Mecanum kinematics
    py::class_<libstp::kinematics::mecanum::MecanumKinematics, libstp::kinematics::IKinematics>(m, "MecanumKinematics")
        .def(py::init<double, double, double>())
        .def("wheelCount", &libstp::kinematics::mecanum::MecanumKinematics::wheelCount)
        .def("inverse", &libstp::kinematics::mecanum::MecanumKinematics::inverse)
        .def("forward", &libstp::kinematics::mecanum::MecanumKinematics::forward);
}