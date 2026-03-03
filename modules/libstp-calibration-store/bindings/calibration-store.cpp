#include <pybind11/pybind11.h>
#include "CalibrationStore.hpp"
namespace py = pybind11;

void init_calibration_store(pybind11::module &m) {
    m.def("store_readings", [](const libstp::calibration_store::CalibrationType type,
                               float whiteThreshold, float blackThreshold) {
        return libstp::calibration_store::CalibrationStore::instance().storeReading(blackThreshold, whiteThreshold, type);
    }, "Stores the readings in the File", py::arg("type"), py::arg("whiteThreshold"), py::arg("blackThreshold"));

    m.def("get_readings", [](const libstp::calibration_store::CalibrationType type) {
        return libstp::calibration_store::CalibrationStore::instance().getReadings(type);
    }, "Get the values of the last reading from the File", py::arg("type"));

    m.def("has_readings", []( const libstp::calibration_store::CalibrationType type) {
        return libstp::calibration_store::CalibrationStore::instance().hasReadings(type);
    }, "Checks if values are present for that reading", py::arg("type"));
}
