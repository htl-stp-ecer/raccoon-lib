#define PYBIND11_DETAILED_ERROR_MESSAGES
#include <pybind11/pybind11.h>

#include "foundation/types.hpp"
#include "hal/IMotor.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

void init_motor(const py::module& m)
{
    // Expose both the injectable interface and the concrete platform-backed type.
    py::class_<libstp::hal::motor::IMotor>(m, "IMotor")
        .def("set_speed", &libstp::hal::motor::IMotor::setSpeed, py::arg("percent"))
        .def("set_velocity", &libstp::hal::motor::IMotor::setVelocity, py::arg("velocity"),
             "Set velocity target in BEMF units - firmware handles PID")
        .def("move_to_position", &libstp::hal::motor::IMotor::moveToPosition,
             py::arg("velocity"), py::arg("goal_position"),
             "Move to absolute position at given velocity")
        .def("move_relative", &libstp::hal::motor::IMotor::moveRelative,
             py::arg("velocity"), py::arg("delta_position"),
             "Move relative position at given velocity")
        .def("get_position", &libstp::hal::motor::IMotor::getPosition)
        .def("get_bemf", &libstp::hal::motor::IMotor::getBemf,
             "Read current BEMF velocity value from firmware")
        .def("is_done", &libstp::hal::motor::IMotor::isDone,
             "Check if position move is complete")
        .def("brake", &libstp::hal::motor::IMotor::brake)
        .def("off", &libstp::hal::motor::IMotor::off,
             "Disable motor completely (no power, no brake — free-spinning)")
        .def("reset_position_counter", &libstp::hal::motor::IMotor::resetPositionCounter,
             "Reset the position counter to zero")
        .def_property_readonly("port", &libstp::hal::motor::IMotor::getPort)
        .def_property_readonly("inverted", &libstp::hal::motor::IMotor::isInverted)
        .def("get_calibration", &libstp::hal::motor::IMotor::getCalibration)
        .def("set_calibration", &libstp::hal::motor::IMotor::setCalibration, py::arg("calibration"));

    py::class_<libstp::hal::motor::Motor, libstp::hal::motor::IMotor>(m, "Motor")
        // Calibration is passed by value so Python can construct and update it
        // through the existing libstp.foundation binding.
        .def(py::init<int, bool, libstp::foundation::MotorCalibration>(), py::arg("port"), py::arg("inverted") = false,
             py::arg("calibration") = libstp::foundation::MotorCalibration{})
        .def_static("disable_all", &libstp::hal::motor::Motor::disableAll)
        .def_static("enable_all", &libstp::hal::motor::Motor::enableAll)
        .def("set_speed", &libstp::hal::motor::Motor::setSpeed, py::arg("percent"))
        .def("set_velocity", &libstp::hal::motor::Motor::setVelocity, py::arg("velocity"))
        .def("move_to_position", &libstp::hal::motor::Motor::moveToPosition,
             py::arg("velocity"), py::arg("goal_position"))
        .def("move_relative", &libstp::hal::motor::Motor::moveRelative,
             py::arg("velocity"), py::arg("delta_position"))
        .def("get_position", &libstp::hal::motor::Motor::getPosition)
        .def("get_bemf", &libstp::hal::motor::Motor::getBemf)
        .def("is_done", &libstp::hal::motor::Motor::isDone)
        .def("brake", &libstp::hal::motor::Motor::brake)
        .def("off", &libstp::hal::motor::Motor::off,
             "Disable motor completely (no power, no brake — free-spinning)")
        .def("reset_position_counter", &libstp::hal::motor::Motor::resetPositionCounter,
             "Reset the position counter to zero")
        .def_property_readonly("port", &libstp::hal::motor::Motor::getPort)
        .def_property_readonly("inverted", &libstp::hal::motor::Motor::isInverted)
        .def("get_calibration", &libstp::hal::motor::Motor::getCalibration)
        .def("set_calibration", &libstp::hal::motor::Motor::setCalibration, py::arg("calibration"),
             "Update motor calibration (including ticks_to_rad)");
}
