//
// Created by eternalRose on 11/17/25.
//

#include "IRSensor.hpp"
#include "hal/Analog.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_ir_sensor(py::module& m) {
    py::class_<libstp::sensors::ir::IRSensor, libstp::hal::analog::AnalogSensor>(m, "IRSensor")
        .def(py::init<const int&, float>(),
             py::arg("port"),
             py::arg("calibrationFactor") = 1.0f)
        .def("setCalibration", &libstp::sensors::ir::IRSensor::setCalibration)
        .def("probabilityOfBlack", &libstp::sensors::ir::IRSensor::probabilityOfBlack)
        .def("probabilityOfWhite", &libstp::sensors::ir::IRSensor::probabilityOfWhite);
}