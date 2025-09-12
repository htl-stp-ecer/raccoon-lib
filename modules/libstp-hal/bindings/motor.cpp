#include <pybind11/pybind11.h>
#include "hal/Motor.hpp"

namespace py = pybind11;

void init_motor(const py::module& m)
{
    py::class_<libstp::hal::motor::Motor>(m, "Motor")
        .def(py::init<int, bool>(), py::arg("port"), py::arg("inverted") = false)
        .def("set_speed", &libstp::hal::motor::Motor::setSpeed, py::arg("percent"))
        .def("get_speed", &libstp::hal::motor::Motor::getPosition)
        .def("brake", &libstp::hal::motor::Motor::brake)
        .def_readwrite("port", &libstp::hal::motor::Motor::port);
}
