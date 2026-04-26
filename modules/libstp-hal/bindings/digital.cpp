#include <pybind11/pybind11.h>
#include "hal/Digital.hpp"

namespace py = pybind11;

void init_digital(const py::module& m)
{
    // Port is read-only at the binding level: mutating it from Python would
    // desync the platform-side PortRegistry without rebinding the underlying
    // hardware channel. Construct a new DigitalSensor instead.
    py::class_<libstp::hal::digital::DigitalSensor>(m, "DigitalSensor")
        .def(py::init<int>(), py::arg("port"))
        .def("read", &libstp::hal::digital::DigitalSensor::read)
        .def_property_readonly("port",
            [](const libstp::hal::digital::DigitalSensor& s) { return s.port; });
}
