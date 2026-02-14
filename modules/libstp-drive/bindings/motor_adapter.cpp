#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "drive/motor_adapter.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

void init_motor_adapter(const py::module& m)
{
    py::class_<libstp::drive::MotorAdapter>(m, "MotorAdapter")
        .def(py::init<libstp::hal::motor::Motor*>(),
             py::arg("motor"))
        .def("set_velocity", [](libstp::drive::MotorAdapter& self, double w_ref, double dt)
        {
            self.setVelocity(w_ref, dt);
        }, py::arg("w_ref"), py::arg("dt"))
        .def("get_velocity", &libstp::drive::MotorAdapter::getVelocity)
        .def("brake", &libstp::drive::MotorAdapter::brake)
        .def("update_encoder_velocity", &libstp::drive::MotorAdapter::updateEncoderVelocity, py::arg("dt"));
}
