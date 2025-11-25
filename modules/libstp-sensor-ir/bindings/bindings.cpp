//
// Created by eternalRose on 11/24/25.
//

#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_ir_sensor(py::module& m);

PYBIND11_MODULE(IRSensor, m) {
    m.doc() = "Python bindings for libstp-sensors-ir";

    init_ir_sensor(m);
}
