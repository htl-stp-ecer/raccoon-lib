//
// Created by eternalRose on 11/17/25.
//

#include "IRSensor.hpp"
#include "hal/Analog.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_ir_sensor(py::module& m) {
    py::class_<libstp::sensors::ir::IRSensor, libstp::hal::analog::AnalogSensor>(m, "IRSensor")
        .def(py::init<const int&>(),
             py::arg("port"))
        .def("setCalibration", &libstp::sensors::ir::IRSensor::setCalibration)
        .def("calibrate", &libstp::sensors::ir::IRSensor::calibrate,
             py::arg("values"))
        .def_readonly("blackThreshold", &libstp::sensors::ir::IRSensor::blackThreshold)
        .def_readonly("whiteThreshold", &libstp::sensors::ir::IRSensor::whiteThreshold)
        .def("probabilityOfBlack", &libstp::sensors::ir::IRSensor::probabilityOfBlack)
        .def("probabilityOfWhite", &libstp::sensors::ir::IRSensor::probabilityOfWhite)
        .def("isOnBlack", &libstp::sensors::ir::IRSensor::isOnBlack)
        .def("isOnWhite", &libstp::sensors::ir::IRSensor::isOnWhite);
}