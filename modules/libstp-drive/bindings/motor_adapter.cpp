#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "drive/motor_adapter.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

void init_motor_adapter(const py::module& m)
{
    py::class_<libstp::drive::MotorCalibration>(m, "MotorCalibration")
        .def(py::init<>())
        .def_readwrite("ff", &libstp::drive::MotorCalibration::ff)
        .def_readwrite("pid", &libstp::drive::MotorCalibration::pid)
        .def_readwrite("deadzone", &libstp::drive::MotorCalibration::deadzone)
        .def_readwrite("max_percent_output", &libstp::drive::MotorCalibration::max_percent_output)
        .def_readwrite("ticks_to_rad", &libstp::drive::MotorCalibration::ticks_to_rad)
        .def_readwrite("vel_lpf_alpha", &libstp::drive::MotorCalibration::vel_lpf_alpha)
        .def_readwrite("invert_meas", &libstp::drive::MotorCalibration::invert_meas)
        .def_readwrite("invert_cmd", &libstp::drive::MotorCalibration::invert_cmd);

    py::class_<libstp::drive::MotorAdapter>(m, "MotorAdapter")
        .def(py::init<libstp::hal::motor::Motor*, const libstp::drive::MotorCalibration&>(),
             py::arg("motor"), py::arg("calibration") = libstp::drive::MotorCalibration{})
        .def("set_velocity_with_accel", [](libstp::drive::MotorAdapter& self, double w_ref, double a_ref, double dt) {
            bool out_saturated;
            self.setVelocityWithAccel(w_ref, a_ref, dt, &out_saturated);
            return out_saturated;
        }, py::arg("w_ref"), py::arg("a_ref"), py::arg("dt"))
        .def("set_velocity", &libstp::drive::MotorAdapter::setVelocity, py::arg("w_ref"), py::arg("dt"))
        .def("set_percent", &libstp::drive::MotorAdapter::setPercent, py::arg("percent"))
        .def("get_velocity", &libstp::drive::MotorAdapter::getVelocity)
        .def("get_raw_percent", &libstp::drive::MotorAdapter::getRawPercent)
        .def("set_calibration", &libstp::drive::MotorAdapter::setCalibration, py::arg("calibration"))
        .def("get_calibration", &libstp::drive::MotorAdapter::getCalibration, py::return_value_policy::reference_internal)
        .def("reset_controller", &libstp::drive::MotorAdapter::resetController)
        .def("brake", &libstp::drive::MotorAdapter::brake)
        .def("motor", static_cast<libstp::hal::motor::Motor& (libstp::drive::MotorAdapter::*)()>(&libstp::drive::MotorAdapter::motor), 
             py::return_value_policy::reference_internal)
        .def("update_encoder_velocity", &libstp::drive::MotorAdapter::updateEncoderVelocity, py::arg("dt"));
}