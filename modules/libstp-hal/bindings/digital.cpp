#include <pybind11/pybind11.h>
#include "hal/Digital.hpp"

namespace py = pybind11;

void init_digital(const py::module& m)
{
    // Keep the Python shape aligned with the C++ wrapper for simple sensor access.
    py::class_<libstp::hal::digital::DigitalSensor>(m, "DigitalSensor")
        .def(py::init<int>(), py::arg("port"))
        .def("read", &libstp::hal::digital::DigitalSensor::read)
        .def_readwrite("port", &libstp::hal::digital::DigitalSensor::port);
}
