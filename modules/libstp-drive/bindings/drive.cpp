#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "drive/drive.hpp"
#include "foundation/types.hpp"
#include "hal/Motor.hpp"

namespace py = pybind11;

void init_drive(const py::module& m)
{
    // Foundation types
    py::class_<libstp::foundation::ChassisVel>(m, "ChassisVel")
        .def(py::init<>())
        .def_readwrite("vx", &libstp::foundation::ChassisVel::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisVel::vy)
        .def_readwrite("w", &libstp::foundation::ChassisVel::w);

    py::class_<libstp::foundation::ChassisState>(m, "ChassisState")
        .def(py::init<>())
        .def_readwrite("vx", &libstp::foundation::ChassisState::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisState::vy)
        .def_readwrite("wz", &libstp::foundation::ChassisState::wz);

    py::class_<libstp::drive::Achieved>(m, "Achieved")
        .def(py::init<>())
        .def_readwrite("body", &libstp::drive::Achieved::body)
        .def_readwrite("saturated_any", &libstp::drive::Achieved::saturated_any)
        .def_readwrite("kinematic_sat_mask", &libstp::drive::Achieved::kinematic_sat_mask)
        .def_readwrite("actuator_sat_mask", &libstp::drive::Achieved::actuator_sat_mask);

    py::class_<libstp::drive::Drive>(m, "Drive")
        .def("set_chassis_limits", &libstp::drive::Drive::setChassisLimits, py::arg("lim"))
        .def("set_wheel_limits", &libstp::drive::Drive::setWheelLimits, py::arg("lim"))
        .def("set_wheel_controller_gains", &libstp::drive::Drive::setWheelControllerGains, py::arg("g"))
        .def("set_wheel_feedforward", &libstp::drive::Drive::setWheelFeedforward, py::arg("ff"))
        .def("set_wheel_deadzone", &libstp::drive::Drive::setWheelDeadzone, py::arg("dz"))
        .def("set_wheel_calibration", &libstp::drive::Drive::setWheelCalibration, 
             py::arg("wheel_index"), py::arg("calibration"))
        .def("set_all_wheel_calibrations", &libstp::drive::Drive::setAllWheelCalibrations, py::arg("calibrations"))
        .def("set_velocity", &libstp::drive::Drive::setVelocity, py::arg("v_body"))
        .def("update", &libstp::drive::Drive::update, py::arg("dt"))
        .def("estimate_state", &libstp::drive::Drive::estimateState)
        .def("wheel_count", &libstp::drive::Drive::wheelCount)
        .def("stop", &libstp::drive::Drive::stop, py::arg("hard") = false);
}