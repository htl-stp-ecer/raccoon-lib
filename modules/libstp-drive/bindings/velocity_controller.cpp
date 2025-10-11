#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "drive/velocity_controller.hpp"

namespace py = pybind11;

void init_velocity_controller(const py::module& m)
{

    py::class_<libstp::drive::VelocityController>(m, "VelocityController")
        .def(py::init<libstp::foundation::PidGains, libstp::foundation::Feedforward>(),
             py::arg("g"), py::arg("ff"))
        .def("set_gains", &libstp::drive::VelocityController::setGains, py::arg("g"))
        .def("set_ff", &libstp::drive::VelocityController::setFF, py::arg("ff"))
        .def("gains", &libstp::drive::VelocityController::gains, py::return_value_policy::reference_internal)
        .def("ff", &libstp::drive::VelocityController::ff, py::return_value_policy::reference_internal)
        .def("compute", [](libstp::drive::VelocityController& self, double w_ref, double a_ref, double w_meas, double dt, double u_max) {
            bool out_saturated;
            double result = self.compute(w_ref, a_ref, w_meas, dt, u_max, &out_saturated);
            return py::make_tuple(result, out_saturated);
        }, py::arg("w_ref"), py::arg("a_ref"), py::arg("w_meas"), py::arg("dt"), py::arg("u_max"))
        .def("reset", &libstp::drive::VelocityController::reset);
}