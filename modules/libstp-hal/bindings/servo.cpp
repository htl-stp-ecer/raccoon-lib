#include <pybind11/pybind11.h>
#include "hal/Servo.hpp"

namespace py = pybind11;

void init_servo(const py::module& m)
{
    // Servo only exposes the command-oriented wrapper state, not any platform internals.
    py::class_<libstp::hal::servo::Servo>(m, "Servo")
        .def(py::init<int>(), py::arg("port"))
        .def("set_position", &libstp::hal::servo::Servo::setPosition, py::arg("position"))
        .def("get_position", &libstp::hal::servo::Servo::getPosition)
        .def("enable", &libstp::hal::servo::Servo::enable)
        .def("disable", &libstp::hal::servo::Servo::disable)
        .def_property_readonly("port", [](const libstp::hal::servo::Servo& s) { return s.port; })
        .def_static("fully_disable_all", &libstp::hal::servo::Servo::fullyDisableAll)
        .def("set_smooth_position", &libstp::hal::servo::Servo::setSmoothPosition,
             py::arg("target_deg"), py::arg("speed_deg_per_sec"), py::arg("easing") = 3);
}
