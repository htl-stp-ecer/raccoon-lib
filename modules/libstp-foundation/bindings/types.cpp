//
// Created by tobias on 10/9/25.
//

#include "foundation/types.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <sstream>

namespace {

std::string chassis_cmd_to_string(const libstp::foundation::ChassisCmd& cmd)
{
    std::ostringstream oss;
    oss << "ChassisCmd(vx=" << cmd.vx << ", vy=" << cmd.vy << ", wz=" << cmd.wz << ")";
    return oss.str();
}

std::string chassis_state_to_string(const libstp::foundation::ChassisState& state)
{
    std::ostringstream oss;
    oss << "ChassisState(vx=" << state.vx << ", vy=" << state.vy << ", wz=" << state.wz << ")";
    return oss.str();
}

std::string pose_to_string(const libstp::foundation::Pose& pose)
{
    std::ostringstream oss;
    oss << "Pose(position=[" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
        << "], orientation=[" << pose.orientation.w() << ", " << pose.orientation.x() << ", "
        << pose.orientation.y() << ", " << pose.orientation.z() << "])";
    return oss.str();
}

} // namespace

namespace py = pybind11;

void init_types(const py::module& m)
{
    py::class_<libstp::foundation::ChassisCmd>(m, "ChassisCmd")
        .def(py::init<double, double, double>())
        .def_readwrite("vx", &libstp::foundation::ChassisCmd::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisCmd::vy)
        .def_readwrite("wz", &libstp::foundation::ChassisCmd::wz)
        .def("__repr__", &chassis_cmd_to_string)
        .def("__str__", &chassis_cmd_to_string);

    py::class_<libstp::foundation::ChassisState>(m, "ChassisState")
        .def(py::init<double, double, double>())
        .def_readwrite("vx", &libstp::foundation::ChassisState::vx)
        .def_readwrite("vy", &libstp::foundation::ChassisState::vy)
        .def_readwrite("wz", &libstp::foundation::ChassisState::wz)
        .def("__repr__", &chassis_state_to_string)
        .def("__str__", &chassis_state_to_string);

    py::class_<libstp::foundation::Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("position", &libstp::foundation::Pose::position)
        .def_property(
            "orientation",
            [](const libstp::foundation::Pose& pose) {
                const auto& q = pose.orientation;
                return py::make_tuple(q.w(), q.x(), q.y(), q.z());
            },
            [](libstp::foundation::Pose& pose, const py::sequence& value) {
                if (py::len(value) != 4) {
                    throw py::value_error("orientation expects an iterable of length 4 (w, x, y, z)");
                }
                pose.orientation =
                    libstp::foundation::Quaternionf(value[0].cast<float>(),
                                                    value[1].cast<float>(),
                                                    value[2].cast<float>(),
                                                    value[3].cast<float>());
            })
        .def("__repr__", &pose_to_string)
        .def("__str__", &pose_to_string);
}
