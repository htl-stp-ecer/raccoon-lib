#include <pybind11/pybind11.h>
#include "drive/rate_limiter.hpp"

namespace py = pybind11;

void init_rate_limiter(const py::module& m)
{
    py::class_<libstp::drive::RateLimiter>(m, "RateLimiter")
        .def(py::init<double>(), py::arg("max_rate") = 0.0)
        .def("set_max_rate", &libstp::drive::RateLimiter::setMaxRate, py::arg("r"))
        .def("max_rate", &libstp::drive::RateLimiter::maxRate)
        .def("step", [](const libstp::drive::RateLimiter& self, double target, double current_prev, double dt) {
            double out_accel;
            double result = self.step(target, current_prev, dt, out_accel);
            return py::make_tuple(result, out_accel);
        }, py::arg("target"), py::arg("current_prev"), py::arg("dt"));
}