//
// Python bindings for the IOdometryBridge / OdometryBridge HAL types.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>
#include "hal/IOdometryBridge.hpp"
#include "hal/OdometryBridge.hpp"

namespace py = pybind11;

void init_odometry_bridge(const py::module& m)
{
    using namespace libstp::hal::odometry_bridge;

    py::class_<IOdometryBridge, std::shared_ptr<IOdometryBridge>>(
            m, "IOdometryBridge",
            "Abstract interface for coprocessor odometry communication.");

    py::class_<OdometryBridge, IOdometryBridge, std::shared_ptr<OdometryBridge>>(
            m, "OdometryBridge",
            "Platform-provided odometry bridge for coprocessor communication.")
        .def(py::init<>());
}
