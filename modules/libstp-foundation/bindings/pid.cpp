//
// Created by tobias on 6/8/25.
//

#include "foundation/pid.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_pid(const py::module& m)
{
    py::class_<libstp::utility::PIDController>(m, "PIDController", R"pbdoc(
            A PID controller for managing control loops.

            This class provides methods for tuning and executing a PID control loop
            with configurable proportional, integral, and derivative gains.
        )pbdoc")
        .def(py::init<float, float, float>(),
             py::arg("Kp"), py::arg("Ki"), py::arg("Kd"),
             R"pbdoc(
                     Initialize a PIDController.

                     Args:
                         Kp (float): Proportional gain.
                         Ki (float): Integral gain.
                         Kd (float): Derivative gain.
                 )pbdoc")
        .def("calculate",
             &libstp::utility::PIDController::calculate,
             py::arg("error"),
             R"pbdoc(
                     Calculate the PID output for a given error.

                     Args:
                         error (float): The current error in the system.

                     Returns:
                         float: The calculated PID output, clamped to the specified range.
                 )pbdoc");
}
