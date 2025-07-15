//
// Created by tobias on 6/8/25.
//
#include "datatype/functions.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

void init_functions(py::module& m)
{
    m.def("for_time", libstp::datatype::forTime, R"pbdoc(Execute a function for a certain amount of time.
        Args:
            time (float): The duration in time units.
        Returns:
            None
        )pbdoc", py::arg("time"));

    m.def("for_seconds", libstp::datatype::forSeconds, R"pbdoc(Execute a function for a certain amount of seconds.
        Args:
            seconds (float): The duration in seconds.
        Returns:
            None
        )pbdoc", py::arg("seconds"));

    m.def("for_ticks", &libstp::datatype::forTicks, R"pbdoc(Execute a function for a certain amount of ticks.
        Args:
            ticks (int): The number of ticks.
        Returns:
            ConditionalFunction: A function that returns a MotorTicksConditionalResult.

        )pbdoc", py::arg("ticks"));

    m.def("for_distance", libstp::datatype::forDistance, R"pbdoc(Execute a function for a certain distance.
        Args:
            distance (float): The distance to cover.
        Returns:
            None
        )pbdoc", py::arg("distance"));

    m.def("for_cw_rotation", libstp::datatype::forCWRotation, R"pbdoc(Execute a function for a clockwise rotation.
        Args:
            rotation_degrees (float): The rotation in degrees.
        Returns:
            None
        )pbdoc", py::arg("rotation_degrees"));

    m.def("for_ccw_rotation", libstp::datatype::forCCWRotation, R"pbdoc(Execute a function for a counter-clockwise rotation.
        Args:
            rotation_degrees (float): The rotation in degrees.
        Returns:
            None
        )pbdoc", py::arg("rotation_degrees"));

    m.def("while_true", libstp::datatype::whileTrue, R"pbdoc(Execute a function while a condition is true.
        Args:
            condition (function): The condition to evaluate.
        Returns:
            None
        )pbdoc", py::arg("condition"));

    m.def("while_false", libstp::datatype::whileFalse, R"pbdoc(Execute a function while a condition is false.
        Args:
            condition (function): The condition to evaluate.
        Returns:
            None
        )pbdoc", py::arg("condition"));

    m.def("generator", libstp::datatype::generator, R"pbdoc(Return a speed generator function.
        Args:
            generator (function): The generator function.
        Returns:
            function: A speed generator function.
        )pbdoc", py::arg("generator"), py::return_value_policy::reference);

    m.def("constant", libstp::datatype::constant, R"pbdoc(Return a constant speed function.
        Args:
            speed (float): The constant speed.
        Returns:
            function: A constant speed function.
        )pbdoc", py::arg("speed"));
}
