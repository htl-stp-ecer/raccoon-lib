#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "drive/drive.hpp"
#include "foundation/types.hpp"
#include "kinematics/kinematics.hpp"
#include "hal/IMU.hpp"
#include "hal/Motor.hpp"

#include <sstream>
namespace py = pybind11;

void init_drive(const py::module& m)
{
    py::class_<libstp::drive::AxisVelocityControlConfig>(m, "AxisVelocityControlConfig")
        .def(py::init<>())
        .def(py::init<libstp::foundation::PidGains, libstp::foundation::Feedforward>(),
             py::arg("pid"), py::arg("ff"))
        .def_readwrite("pid", &libstp::drive::AxisVelocityControlConfig::pid)
        .def_readwrite("ff", &libstp::drive::AxisVelocityControlConfig::ff);

    py::class_<libstp::drive::ChassisVelocityControlConfig>(m, "ChassisVelocityControlConfig")
        .def(py::init<>())
        .def_readwrite("vx", &libstp::drive::ChassisVelocityControlConfig::vx)
        .def_readwrite("vy", &libstp::drive::ChassisVelocityControlConfig::vy)
        .def_readwrite("wz", &libstp::drive::ChassisVelocityControlConfig::wz);

    py::class_<libstp::drive::Drive>(m, "Drive")
        // Python hands in an existing kinematics instance; the C++ Drive stores ownership.
        .def(py::init([](libstp::kinematics::IKinematics* kinematics,
                         const libstp::drive::ChassisVelocityControlConfig& vel_config,
                         libstp::hal::imu::IMU& imu)
        {
            return std::make_unique<libstp::drive::Drive>(
                std::unique_ptr<libstp::kinematics::IKinematics>(kinematics),
                vel_config, imu);
        }), py::arg("kinematics"), py::arg("vel_config"),
            py::arg("imu"),
            py::keep_alive<1, 2>(), py::keep_alive<1, 4>())
        .def("set_velocity", &libstp::drive::Drive::setVelocity, py::arg("v_body"))
        // Preserve the existing Python API shape even though C++ returns MotorCommands.
        .def("update", [](libstp::drive::Drive& self, double dt) {
            self.update(dt);  // Ignore MotorCommands return value
        }, py::arg("dt"))
        .def("estimate_state", &libstp::drive::Drive::estimateState)
        .def("wheel_count", &libstp::drive::Drive::wheelCount)
        .def("soft_stop", &libstp::drive::Drive::softStop)
        .def("hard_stop", &libstp::drive::Drive::hardStop)
        .def("set_velocity_control_config", &libstp::drive::Drive::setVelocityControlConfig, py::arg("config"))
        .def("get_velocity_control_config", &libstp::drive::Drive::getVelocityControlConfig,
             py::return_value_policy::copy,
             "Get the current velocity control config (copy)")
        .def("reset_velocity_controllers", &libstp::drive::Drive::resetVelocityControllers)
        .def("get_wheel_radius", &libstp::drive::Drive::getWheelRadius,
             "Get the wheel radius from kinematics in meters")
        .def("apply_power_command", &libstp::drive::Drive::applyPowerCommand,
             py::arg("direction"), py::arg("power_percent"),
             "Command motors at raw open-loop power using kinematics for direction")
        .def("get_motors", [](libstp::drive::Drive& self) {
            auto motors = self.getMotors();
            std::vector<libstp::hal::motor::Motor*> result;
            result.reserve(motors.size());
            for (auto* m : motors) {
                result.push_back(static_cast<libstp::hal::motor::Motor*>(m));
            }
            return result;
        }, py::return_value_policy::reference,
           "Get all drive motors managed by the kinematics");
}
