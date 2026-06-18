#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>
#include "hal/Digital.hpp"

namespace py = pybind11;

void init_digital(const py::module& m)
{
    // shared_ptr holder so the instance can be co-owned by the button
    // singleton without copying or reconstructing it (see libstp-button).
    // Port is read-only at the binding level: mutating it from Python would
    // desync the platform-side PortRegistry without rebinding the underlying
    // hardware channel. Construct a new DigitalSensor instead.
    py::class_<libstp::hal::digital::DigitalSensor,
               std::shared_ptr<libstp::hal::digital::DigitalSensor>>(m, "DigitalSensor")
        .def(py::init<int>(), py::arg("port"))
        .def("read", &libstp::hal::digital::DigitalSensor::read)
        .def_property_readonly("port",
            [](const libstp::hal::digital::DigitalSensor& s) { return s.port; });

    // Composite button: reads pressed when ANY member port is pressed.
    py::class_<libstp::hal::digital::ButtonGroup,
               libstp::hal::digital::DigitalSensor,
               std::shared_ptr<libstp::hal::digital::ButtonGroup>>(m, "ButtonGroup")
        .def(py::init<std::vector<int>>(), py::arg("ports"))
        .def("read", &libstp::hal::digital::ButtonGroup::read)
        .def_property_readonly("ports", &libstp::hal::digital::ButtonGroup::ports);
}
