#include <pybind11/pybind11.h>
#include "hal/Servo.hpp"

namespace py = pybind11;

void init_servo(const py::module& m)
{
    py::class_<libstp::hal::servo::Servo>(m, "Servo")
        .def(py::init<int>(), py::arg("port"))
        .def("set_position", &libstp::hal::servo::Servo::setPosition, py::arg("position"))
        .def("get_position", &libstp::hal::servo::Servo::getPosition)
        .def("enable", &libstp::hal::servo::Servo::enable)
        .def("disable", &libstp::hal::servo::Servo::disable)
        .def_readwrite("port", &libstp::hal::servo::Servo::port)
        .def_static("fully_disable_all", &libstp::hal::servo::Servo::fullyDisableAll);
}
