//
// Created by tobias on 10/9/25.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>

#include "kinematics/mecanum/mecanum.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kinematics_mecanum, m)
{
    m.doc() = "Python bindings for libstp-kinematics_mecanum";

    // Ensure dependent base/types are registered before referencing them
    py::module_::import("raccoon.kinematics");
    py::module_::import("raccoon.hal");

    py::class_<libstp::kinematics::mecanum::MecanumKinematics, libstp::kinematics::IKinematics,
                std::shared_ptr<libstp::kinematics::mecanum::MecanumKinematics>>(m, "MecanumKinematics")
        // Keep motor wrappers alive for at least as long as the kinematics object.
        .def(py::init<libstp::hal::motor::Motor*,
                      libstp::hal::motor::Motor*,
                      libstp::hal::motor::Motor*,
                      libstp::hal::motor::Motor*,
                      double, double, double>(),
             py::arg("front_left_motor"),
             py::arg("front_right_motor"),
             py::arg("back_left_motor"),
             py::arg("back_right_motor"),
             py::arg("wheelbase"),
             py::arg("track_width"),
             py::arg("wheel_radius"),
             py::keep_alive<1, 2>())
        .def("wheel_count", &libstp::kinematics::mecanum::MecanumKinematics::wheelCount)
        .def("apply_command", &libstp::kinematics::mecanum::MecanumKinematics::applyCommand,
             py::arg("cmd"), py::arg("dt"))
        .def("estimate_state", &libstp::kinematics::mecanum::MecanumKinematics::estimateState)
        .def("hard_stop", &libstp::kinematics::mecanum::MecanumKinematics::hardStop)
        .def("reset_encoders", &libstp::kinematics::mecanum::MecanumKinematics::resetEncoders,
             "Reset encoder tracking to prevent stale deltas after odometry reset")
        .def("supports_lateral_motion", &libstp::kinematics::mecanum::MecanumKinematics::supportsLateralMotion,
             "Returns True since mecanum drives can strafe")
        .def("get_wheel_radius", &libstp::kinematics::mecanum::MecanumKinematics::getWheelRadius,
             "Get the wheel radius in meters")
        .def("set_max_wheel_speed", &libstp::kinematics::mecanum::MecanumKinematics::setMaxWheelSpeed,
             py::arg("max_wheel_speed"),
             "Set max wheel speed (rad/s) for desaturation. 0 = disabled.")
        .def("get_max_wheel_speed", &libstp::kinematics::mecanum::MecanumKinematics::getMaxWheelSpeed,
             "Get the max wheel speed limit (rad/s). 0 = disabled.")
        .def("apply_power_command", &libstp::kinematics::mecanum::MecanumKinematics::applyPowerCommand,
             py::arg("direction"), py::arg("power_percent"),
             "Command motors at raw open-loop power using kinematics for direction")
        .def_property_readonly("motors", &libstp::kinematics::mecanum::MecanumKinematics::getMotors,
             "List of motors managed by this kinematics model");
}
