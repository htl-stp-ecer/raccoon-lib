#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "hal/odometry.hpp"
#include "foundation/types.hpp"

namespace py = pybind11;

void init_odometry(py::module& m)
{
    py::class_<libstp::odometry::DistanceFromOrigin>(m, "DistanceFromOrigin")
        .def_readonly("forward", &libstp::odometry::DistanceFromOrigin::forward)
        .def_readonly("lateral", &libstp::odometry::DistanceFromOrigin::lateral)
        .def_readonly("straight_line", &libstp::odometry::DistanceFromOrigin::straight_line)
        .def("__repr__", [](const libstp::odometry::DistanceFromOrigin& d) {
            return "DistanceFromOrigin(forward=" + std::to_string(d.forward)
                + ", lateral=" + std::to_string(d.lateral)
                + ", straight_line=" + std::to_string(d.straight_line) + ")";
        });

    py::enum_<libstp::odometry::OdometrySource>(m, "OdometrySource")
        .value("INTERNAL", libstp::odometry::OdometrySource::Internal)
        .value("CALIBRATION_BOARD", libstp::odometry::OdometrySource::CalibrationBoard);

    py::class_<libstp::odometry::IOdometry,
                std::shared_ptr<libstp::odometry::IOdometry>>(m, "IOdometry")
        .def("update", &libstp::odometry::IOdometry::update,
             py::arg("dt"))
        .def("get_pose", &libstp::odometry::IOdometry::getPose)
        .def("get_distance_from_origin", &libstp::odometry::IOdometry::getDistanceFromOrigin)
        .def("get_heading", &libstp::odometry::IOdometry::getHeading)
        .def("get_heading_error", &libstp::odometry::IOdometry::getHeadingError,
             py::arg("target_heading_rad"))
        .def("get_absolute_heading", &libstp::odometry::IOdometry::getAbsoluteHeading)
        .def("get_path_length", &libstp::odometry::IOdometry::getPathLength)
        .def("reset", py::overload_cast<>(&libstp::odometry::IOdometry::reset))
        // Source introspection + internal (cheap) estimate, exposed so the
        // accurate external calibration board can be used to tune the internal
        // dead reckoning. set/get_preferred_source() control whether callers
        // want the calibration board considered at all; get_active_source()
        // reports which source currently backs get_pose()/get_heading(); the
        // get_internal_* accessors always return the on-board STM32 estimate
        // regardless of the active source.
        .def("set_preferred_source", &libstp::odometry::IOdometry::setPreferredSource,
             py::arg("source"))
        .def("get_preferred_source", &libstp::odometry::IOdometry::getPreferredSource)
        .def("get_active_source", &libstp::odometry::IOdometry::getActiveSource)
        .def("get_internal_pose", &libstp::odometry::IOdometry::getInternalPose)
        .def("get_internal_heading", &libstp::odometry::IOdometry::getInternalHeading)
        .def("get_internal_distance_from_origin",
             &libstp::odometry::IOdometry::getInternalDistanceFromOrigin);
}
