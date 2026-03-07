#include "ETSensor.hpp"
#include "hal/Analog.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_et_sensor(py::module& m) {
    py::class_<libstp::sensors::et::ETSensor, libstp::hal::analog::AnalogSensor>(m, "ETSensor")
        .def(py::init<int>(),
             py::arg("port"))
        .def("raw", &libstp::sensors::et::ETSensor::raw);
}
