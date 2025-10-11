//
// Created by tobias on 10/9/25.
//

#include "foundation/types.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_types(const py::module& m)
{
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
}