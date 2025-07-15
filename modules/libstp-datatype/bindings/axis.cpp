//
// Created by tobias on 6/8/25.
//
#include "datatype/axis.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_axis(const py::module& m)
{
    py::enum_<libstp::datatype::Axis>(m, "Axis", "Represents the axis of the robot")
        .value("X", libstp::datatype::X, "Represents the X-axis")
        .value("Y", libstp::datatype::Y, "Represents the Y-axis")
        .value("Z", libstp::datatype::Z, "Represents the Z-axis")
        .export_values();

    py::enum_<libstp::datatype::Direction>(m, "Direction", "Represents the direction of the robot")
        .value("Normal", libstp::datatype::Direction::Forward, "Represents the normal direction")
        .value("Reversed", libstp::datatype::Direction::Backward, "Represents the reversed direction")
        .export_values();
}
