//
// Created by eternalRose on 11/17/25.
//

#include "../include/IRSensor.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;
void init_ir_sensor(const py::module& m) {
    py::class_<libstp::sensors::ir::IRSensor>(m, "IRSensor")
        .def(py::init<const int&, float>(),
             py::arg("port"),
             py::arg("calibrationFactor"))
        .def("setCalibration", &libstp::sensors::ir::IRSensor::setCalibration )
        .def("read", &libstp::sensors::ir::IRSensor::read);
}