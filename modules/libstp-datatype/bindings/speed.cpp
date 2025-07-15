//
// Created by tobias on 6/8/25.
//
#include "datatype/speed.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

void init_speed(const py::module& m)
{
    py::class_<libstp::datatype::Speed>(m, "Speed", "Represents a speed with forward and angular components")
        .def(py::init<float, float>(),
             py::arg("forward_speed_m_per_s"), py::arg("angular_speed_deg_per_s"),
             "Initialize with forward and angular speeds")
        .def(py::init<float, float, float>(),
             py::arg("forward_speed_m_per_s"), py::arg("strafe_speed_m_per_s"), py::arg("angular_speed_deg_per_s"),
             "Initialize with forward, strafe, and angular speeds")
        .def_property_readonly("forward_percent", [](const libstp::datatype::Speed& self) { return self.forwardPercent; },
                               "Get the forward speed as a percentage")
        .def_property_readonly("angular_percent", [](const libstp::datatype::Speed& self) { return self.angularPercent; },
                               "Get the angular speed as a percentage")
        .def_property_readonly("strafe_percent", [](const libstp::datatype::Speed& self) { return self.strafePercent; },
                               "Get the strafe speed as a percentage")
        .def("backward", &libstp::datatype::Speed::backward, "Get a speed representing backward movement")
        .def_static("wheels", &libstp::datatype::Speed::wheels, "Create a speed based on wheel velocities")
        .def_static("stop", &libstp::datatype::Speed::stop, "Get a speed representing no movement")
        .def("__str__", [](const libstp::datatype::Speed& self)
        {
            return "Speed(forward=" + std::to_string(self.forwardPercent) +
                ", strafe=" + std::to_string(self.strafePercent) +
                ", angular=" + std::to_string(self.angularPercent) + ")";
        })
        .def("__repr__", [](const libstp::datatype::Speed& self)
        {
            return "<Speed(forward_percent=" + std::to_string(self.forwardPercent) +
                ", strafe_percent=" + std::to_string(self.strafePercent) +
                ", angular_percent=" + std::to_string(self.angularPercent) + ")>";
        })
        .def_readonly_static("Slowest", &libstp::datatype::Speed::Slowest, "Predefined slowest speed")
        .def_readonly_static("Slow", &libstp::datatype::Speed::Slow, "Predefined slow speed")
        .def_readonly_static("Medium", &libstp::datatype::Speed::Medium, "Predefined medium speed")
        .def_readonly_static("Fast", &libstp::datatype::Speed::Fast, "Predefined fast speed")
        .def_readonly_static("Fastest", &libstp::datatype::Speed::Fastest, "Predefined fastest speed");
}
