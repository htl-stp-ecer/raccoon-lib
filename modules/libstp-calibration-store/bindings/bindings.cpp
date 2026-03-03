#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_calibration_store(py::module& m);
void init_calibration_store_type(const py::module &m);

PYBIND11_MODULE(calibration_store, m) {
    m.doc() = "Python bindings for libstp-calibration-store";

    // The module exposes singleton-backed free functions rather than a Python class wrapper.
    init_calibration_store(m);
    init_calibration_store_type(m);
}
