//
// Created by eternalRose on 1/19/26.
//

#include "CalibrationType.hpp"
#include "pybind11/pybind11.h"
namespace py = pybind11;

void init_calibration_store_type(const pybind11::module &m) {
    py::enum_<libstp::calibration_store::CalibrationType>(m, "CalibrationType")
        .value("IR_SENSOR", libstp::calibration_store::CalibrationType::IR_SENSOR)
        .export_values();
}