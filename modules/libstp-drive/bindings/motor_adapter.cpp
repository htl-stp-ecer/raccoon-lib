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
        .def("set_velocity_with_accel", [](libstp::drive::MotorAdapter& self, double w_ref, double a_ref, double dt)
        {
            bool out_saturated;
            self.setVelocityWithAccel(w_ref, a_ref, dt, &out_saturated);
            return out_saturated;
        }, py::arg("w_ref"), py::arg("a_ref"), py::arg("dt"))
        .def("set_velocity", &libstp::drive::MotorAdapter::setVelocity, py::arg("w_ref"), py::arg("dt"))
        .def("set_percent", &libstp::drive::MotorAdapter::setPercent, py::arg("percent"))
        .def("get_velocity", &libstp::drive::MotorAdapter::getVelocity)
        .def("get_raw_percent", &libstp::drive::MotorAdapter::getRawPercent)
        .def("reset_controller", &libstp::drive::MotorAdapter::resetController)
        .def("brake", &libstp::drive::MotorAdapter::brake)
        .def("update_encoder_velocity", &libstp::drive::MotorAdapter::updateEncoderVelocity, py::arg("dt"));
}
