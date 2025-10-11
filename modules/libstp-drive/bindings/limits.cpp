#include <pybind11/pybind11.h>
#include "drive/limits.hpp"

namespace py = pybind11;

void init_limits(const py::module& m)
{
    py::class_<libstp::drive::MotionLimits>(m, "MotionLimits")
        .def(py::init([](const double max_v, const double max_omega)
        {
            libstp::drive::MotionLimits lim;
            lim.max_v = max_v;
            lim.max_omega = max_omega;
            return lim;
        }), py::arg("max_v"), py::arg("max_omega"))
        .def_readwrite("max_v", &libstp::drive::MotionLimits::max_v)
        .def_readwrite("max_omega", &libstp::drive::MotionLimits::max_omega);

    py::class_<libstp::drive::WheelLimits>(m, "WheelLimits")
        .def(py::init([](const double max_w, const double max_w_dot)
        {
            libstp::drive::WheelLimits lim;
            lim.max_w = max_w;
            lim.max_w_dot = max_w_dot;
            return lim;
        }), py::arg("max_w"), py::arg("max_w_dot"))
        .def_readwrite("max_w", &libstp::drive::WheelLimits::max_w)
        .def_readwrite("max_w_dot", &libstp::drive::WheelLimits::max_w_dot);
}
