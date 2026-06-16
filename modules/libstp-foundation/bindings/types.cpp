#include "foundation/types.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <sstream>

namespace {

std::string chassis_velocity_to_string(const libstp::foundation::ChassisVelocity& vel)
{
    std::ostringstream oss;
    oss << "ChassisVelocity(vx=" << vel.vx << ", vy=" << vel.vy << ", wz=" << vel.wz << ")";
    return oss.str();
}

std::string velocity_command_gain_to_string(const libstp::foundation::VelocityCommandGain& g)
{
    std::ostringstream oss;
    oss << "VelocityCommandGain(vx=" << g.vx << ", vy=" << g.vy << ", wz=" << g.wz << ")";
    return oss.str();
}

std::string pose_to_string(const libstp::foundation::Pose& pose)
{
    std::ostringstream oss;
    oss << "Pose(position=[" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
        << "], heading=" << pose.heading << ")";
    return oss.str();
}

} // namespace

namespace py = pybind11;

void init_types(const py::module& m)
{
    py::class_<libstp::foundation::ChassisVelocity>(m, "ChassisVelocity")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("vx"), py::arg("vy"), py::arg("wz"))
        .def_readwrite("vx", &libstp::foundation::ChassisVelocity::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisVelocity::vy)
        .def_readwrite("wz", &libstp::foundation::ChassisVelocity::wz)
        .def("__repr__", &chassis_velocity_to_string)
        .def("__str__", &chassis_velocity_to_string);

    py::class_<libstp::foundation::VelocityCommandGain>(m, "VelocityCommandGain")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("vx") = 1.0, py::arg("vy") = 1.0, py::arg("wz") = 1.0)
        .def_readwrite("vx", &libstp::foundation::VelocityCommandGain::vx)
        .def_readwrite("vy", &libstp::foundation::VelocityCommandGain::vy)
        .def_readwrite("wz", &libstp::foundation::VelocityCommandGain::wz)
        .def("__repr__", &velocity_command_gain_to_string)
        .def("__str__", &velocity_command_gain_to_string);

    py::class_<libstp::foundation::Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("position", &libstp::foundation::Pose::position)
        .def_readwrite("heading", &libstp::foundation::Pose::heading)
        .def("__repr__", &pose_to_string)
        .def("__str__", &pose_to_string);
}
