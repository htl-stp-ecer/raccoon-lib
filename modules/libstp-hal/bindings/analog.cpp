//
// Created by tobias on 6/1/25.
//
#include <pybind11/pybind11.h>
#include "hal/Analog.hpp"

namespace py = pybind11;

void init_analog(const py::module& m)
{
    py::class_<libstp::hal::analog::AnalogSensor>(m, "AnalogSensor")
        .def(py::init<int>(), py::arg("port"))
        .def("read", &libstp::hal::analog::AnalogSensor::read)
        .def_readwrite("port", &libstp::hal::analog::AnalogSensor::port);
}
