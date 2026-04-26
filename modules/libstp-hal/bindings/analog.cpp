#include <pybind11/pybind11.h>
#include "hal/Analog.hpp"

namespace py = pybind11;

void init_analog(const py::module& m)
{
    // Port is read-only at the binding level: mutating it from Python would
    // desync the platform-side PortRegistry without rebinding the underlying
    // hardware channel. Construct a new AnalogSensor instead.
    py::class_<libstp::hal::analog::AnalogSensor>(m, "AnalogSensor")
        .def(py::init<int>(), py::arg("port"))
        .def("read", &libstp::hal::analog::AnalogSensor::read)
        .def_property_readonly("port",
            [](const libstp::hal::analog::AnalogSensor& s) { return s.port; });
}
