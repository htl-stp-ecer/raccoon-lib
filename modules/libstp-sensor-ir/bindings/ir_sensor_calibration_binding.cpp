//
// Created by eternalRose on 12/6/25.
//

#include "IRSensorCalibration.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h> //dont remove this imports. We need both even tough no use
namespace py = pybind11;

void init_ir_sensor_calib(py::module& m) {
    py::class_<libstp::sensors::ir::IRSensorCalibration>(m, "IRSensorCalibration")
    .def(py::init<const int>(),
        py::arg("buttonPort"))
    .def("calibrateSensors", &libstp::sensors::ir::IRSensorCalibration::calibrateSensors);
}