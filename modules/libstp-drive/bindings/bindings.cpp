//
// Created by tobias on 9/11/25.
//
#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_drive(const py::module& m);
void init_motor_adapter(const py::module& m);
void init_velocity_controller(const py::module& m);

PYBIND11_MODULE(drive, m) {
    m.doc() = "Python bindings for libstp-drive";

    init_velocity_controller(m);
    init_motor_adapter(m);
    init_drive(m);
}