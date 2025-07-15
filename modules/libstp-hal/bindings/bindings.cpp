//
// Created by tobias on 5/19/25.
//
#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_imu(const py::module& m);
void init_analog(const py::module& m);
void init_digital(const py::module& m);
void init_motor(const py::module& m);
void init_servo(const py::module& m);

PYBIND11_MODULE(hal, m) {
    m.doc() = "Python bindings for libstp-hal";

    init_imu(m);
    init_analog(m);
    init_digital(m);
    init_motor(m);
    init_servo(m);
}
