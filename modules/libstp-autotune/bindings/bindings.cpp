#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "autotune/characterize_drive.hpp"
#include "autotune/types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(autotune, m)
{
    m.doc() = "Python bindings for libstp-autotune (DriveCharacterizer)";

    using namespace libstp::autotune;

    // ------------------------------------------------------------------
    // AxisResult — read-only from Python
    // ------------------------------------------------------------------
    py::class_<AxisResult>(m, "AxisResult")
        .def(py::init<>())
        .def_readonly("max_velocity", &AxisResult::max_velocity,
                      "Peak velocity at full power (m/s or rad/s).")
        .def_readonly("acceleration", &AxisResult::acceleration,
                      "Acceleration from 10%%–90%% of max velocity (m/s² or rad/s²).")
        .def_readonly("deceleration", &AxisResult::deceleration,
                      "Deceleration from 90%%–10%% during coast-down (m/s² or rad/s²).")
        .def("__repr__", [](const AxisResult& r) {
            std::ostringstream oss;
            oss << "AxisResult(max_velocity=" << r.max_velocity
                << ", acceleration=" << r.acceleration
                << ", deceleration=" << r.deceleration << ")";
            return oss.str();
        });

    // ------------------------------------------------------------------
    // CharacterizeConfig — read-write from Python
    // ------------------------------------------------------------------
    py::class_<CharacterizeConfig>(m, "CharacterizeConfig")
        .def(py::init<>())
        .def_readwrite("power_percent", &CharacterizeConfig::power_percent,
                       "Raw PWM percentage (1–100).")
        .def_readwrite("trials", &CharacterizeConfig::trials,
                       "Number of trials per axis; median is taken.")
        .def_readwrite("accel_timeout", &CharacterizeConfig::accel_timeout,
                       "Maximum seconds to wait for the acceleration phase.")
        .def_readwrite("decel_timeout", &CharacterizeConfig::decel_timeout,
                       "Maximum seconds to record the coast-down phase.")
        .def_readwrite("sample_hz", &CharacterizeConfig::sample_hz,
                       "Position sampling rate in Hz (default 500).");

    // ------------------------------------------------------------------
    // DriveCharacterizer
    // ------------------------------------------------------------------
    py::class_<DriveCharacterizer>(m, "DriveCharacterizer")
        .def(py::init([](libstp::drive::Drive&        drive,
                         libstp::odometry::IOdometry& odometry)
             {
                 return std::make_unique<DriveCharacterizer>(drive, odometry);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::keep_alive<1, 2>(),  // keep drive alive while characterizer is alive
             py::keep_alive<1, 3>(),  // keep odometry alive while characterizer is alive
             "Construct a DriveCharacterizer.\n\n"
             "Parameters\n----------\n"
             "drive : Drive\n    Chassis drive controller.\n"
             "odometry : IOdometry\n    Odometry source.\n")

        .def("characterize_axis",
             &DriveCharacterizer::characterizeAxis,
             py::arg("axis"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Characterize a single axis (\"forward\", \"lateral\", or \"angular\").\n\n"
             "Releases the GIL for the full sampling loop.\n\n"
             "Returns\n-------\n"
             "AxisResult\n    Median max_velocity, acceleration, and deceleration.")

        .def("characterize",
             &DriveCharacterizer::characterize,
             py::arg("axes"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Characterize multiple axes in sequence.\n\n"
             "Releases the GIL for the full sampling loop.\n\n"
             "Returns\n-------\n"
             "dict[str, AxisResult]\n    Map from axis name to AxisResult.");
}
