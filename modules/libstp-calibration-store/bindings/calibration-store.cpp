#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "CalibrationStore.hpp"
namespace py = pybind11;

void init_calibration_store(pybind11::module &m) {
    m.def("store_readings", [](const libstp::calibration_store::CalibrationType type,
                               float whiteThreshold, float blackThreshold,
                               const std::string& set_name) {
        return libstp::calibration_store::CalibrationStore::instance().storeReading(blackThreshold, whiteThreshold, type, set_name);
    }, "Stores the readings in the File", py::arg("type"), py::arg("whiteThreshold"), py::arg("blackThreshold"),
       py::arg("set_name") = "default");

    m.def("get_readings", [](const libstp::calibration_store::CalibrationType type,
                              const std::string& set_name) {
        return libstp::calibration_store::CalibrationStore::instance().getReadings(type, set_name);
    }, "Get the values of the last reading from the File", py::arg("type"),
       py::arg("set_name") = "default");

    m.def("has_readings", [](const libstp::calibration_store::CalibrationType type,
                              const std::string& set_name) {
        return libstp::calibration_store::CalibrationStore::instance().hasReadings(type, set_name);
    }, "Checks if values are present for that reading", py::arg("type"),
       py::arg("set_name") = "default");

    m.def("get_set_names", [](const libstp::calibration_store::CalibrationType type) {
        return libstp::calibration_store::CalibrationStore::instance().getSetNames(type);
    }, "Returns the names of all calibration sets for a type", py::arg("type"));
}
