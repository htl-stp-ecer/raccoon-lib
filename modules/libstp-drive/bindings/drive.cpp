#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "drive/drive.hpp"
#include "foundation/types.hpp"
#include "kinematics/kinematics.hpp"
#include "hal/Motor.hpp"

#include <sstream>
namespace py = pybind11;

void init_drive(const py::module& m)
{
    py::class_<libstp::drive::Drive>(m, "Drive")
        .def(py::init([](libstp::kinematics::IKinematics* kinematics, const libstp::drive::MotionLimits& chassis_lim)
        {
            return std::make_unique<libstp::drive::Drive>(std::unique_ptr<libstp::kinematics::IKinematics>(kinematics),
                                                          chassis_lim);
        }), py::arg("kinematics"), py::arg("chassis_lim"), py::keep_alive<1, 2>())
        .def("set_velocity", &libstp::drive::Drive::setVelocity, py::arg("v_body"))
        .def("update", &libstp::drive::Drive::update, py::arg("dt"))
        .def("estimate_state", &libstp::drive::Drive::estimateState)
        .def("wheel_count", &libstp::drive::Drive::wheelCount)
        .def("soft_stop", &libstp::drive::Drive::softStop)
        .def("hard_stop", &libstp::drive::Drive::hardStop)
        .def("get_wheel_radius", &libstp::drive::Drive::getWheelRadius,
             "Get the wheel radius from kinematics in meters")
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
