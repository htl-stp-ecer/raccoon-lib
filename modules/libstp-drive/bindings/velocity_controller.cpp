#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "drive/velocity_controller.hpp"

namespace py = pybind11;

void init_velocity_controller(const py::module& m)
{
    py::class_<libstp::drive::PidGains>(m, "PidGains")
        .def(py::init<>())
        .def_readwrite("kp", &libstp::drive::PidGains::kp)
        .def_readwrite("ki", &libstp::drive::PidGains::ki)
        .def_readwrite("kd", &libstp::drive::PidGains::kd);

    py::class_<libstp::drive::Feedforward>(m, "Feedforward")
        .def(py::init<>())
        .def_readwrite("kS", &libstp::drive::Feedforward::kS)
        .def_readwrite("kV", &libstp::drive::Feedforward::kV)
        .def_readwrite("kA", &libstp::drive::Feedforward::kA);

    py::class_<libstp::drive::Deadzone>(m, "Deadzone")
        .def(py::init<>())
        .def_readwrite("enable", &libstp::drive::Deadzone::enable)
        .def_readwrite("zero_window_percent", &libstp::drive::Deadzone::zero_window_percent)
        .def_readwrite("start_percent", &libstp::drive::Deadzone::start_percent)
        .def_readwrite("release_percent", &libstp::drive::Deadzone::release_percent);

    py::class_<libstp::drive::VelocityController>(m, "VelocityController")
        .def(py::init<libstp::drive::PidGains, libstp::drive::Feedforward, libstp::drive::Deadzone>(),
             py::arg("g"), py::arg("ff"), py::arg("dz"))
        .def("set_gains", &libstp::drive::VelocityController::setGains, py::arg("g"))
        .def("set_ff", &libstp::drive::VelocityController::setFF, py::arg("ff"))
        .def("set_deadzone", &libstp::drive::VelocityController::setDeadzone, py::arg("dz"))
        .def("gains", &libstp::drive::VelocityController::gains, py::return_value_policy::reference_internal)
        .def("ff", &libstp::drive::VelocityController::ff, py::return_value_policy::reference_internal)
        .def("deadzone", &libstp::drive::VelocityController::deadzone, py::return_value_policy::reference_internal)
        .def("compute", [](libstp::drive::VelocityController& self, double w_ref, double a_ref, double w_meas, double dt, double u_max) {
            bool out_saturated;
            double result = self.compute(w_ref, a_ref, w_meas, dt, u_max, &out_saturated);
            return py::make_tuple(result, out_saturated);
        }, py::arg("w_ref"), py::arg("a_ref"), py::arg("w_meas"), py::arg("dt"), py::arg("u_max"))
        .def("reset", &libstp::drive::VelocityController::reset);
}