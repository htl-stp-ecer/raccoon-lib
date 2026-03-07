#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_et_sensor(py::module& m);

PYBIND11_MODULE(sensor_et, m) {
    m.doc() = "Python bindings for libstp-sensor-et";

    init_et_sensor(m);
}
