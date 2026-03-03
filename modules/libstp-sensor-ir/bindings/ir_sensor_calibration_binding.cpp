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
    .def_static("calibrateSensors", &libstp::sensors::ir::IRSensorCalibration::calibrateSensors,
                py::arg("sensors"), py::arg("durationSeconds") = 5.0f,
                py::arg("usePre") = false, py::arg("set_name") = "default")
    .def_static("collectValues", &libstp::sensors::ir::IRSensorCalibration::collectValues);
}
