#include <pybind11/pybind11.h>
namespace py = pybind11;

// Submodule initializers live in separate files so the binding surface can grow
// without turning the main module entrypoint into a single large translation unit.
void init_imu(const py::module& m);
void init_analog(const py::module& m);
void init_digital(const py::module& m);
void init_motor(const py::module& m);
void init_servo(const py::module& m);
void init_odometry_bridge(const py::module& m);
void init_platform(py::module& m);

PYBIND11_MODULE(hal, m) {
    m.doc() = "Python bindings for libstp-hal";

    // Motor exposes foundation::MotorCalibration, so load the sibling module first.
    py::module::import("raccoon.foundation");

    init_imu(m);
    init_analog(m);
    init_digital(m);
    init_motor(m);
    init_servo(m);
    init_odometry_bridge(m);
    init_platform(m);
}
