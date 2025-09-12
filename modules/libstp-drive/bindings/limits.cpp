#include <pybind11/pybind11.h>
#include "drive/limits.hpp"

namespace py = pybind11;

void init_limits(const py::module& m)
{
    py::class_<libstp::drive::MotionLimits>(m, "MotionLimits")
        .def(py::init<>())
        .def_readwrite("max_v", &libstp::drive::MotionLimits::max_v)
        .def_readwrite("max_omega", &libstp::drive::MotionLimits::max_omega);

    py::class_<libstp::drive::WheelLimits>(m, "WheelLimits")
        .def(py::init<>())
        .def_readwrite("max_w", &libstp::drive::WheelLimits::max_w)
        .def_readwrite("max_w_dot", &libstp::drive::WheelLimits::max_w_dot);
}