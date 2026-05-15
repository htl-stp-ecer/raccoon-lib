#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_imu(const py::module& m);
void init_analog(const py::module& m);
void init_digital(const py::module& m);
void init_motor(const py::module& m);
void init_servo(const py::module& m);
void init_odometry(py::module& m);
void init_platform(py::module& m);

PYBIND11_MODULE(hal, m) {
    m.doc() = "Python bindings for libstp-hal";

    py::module::import("raccoon.foundation");

    init_imu(m);
    init_analog(m);
    init_digital(m);
    init_motor(m);
    init_servo(m);
    init_odometry(m);
    init_platform(m);
}
