//
// Created by tobias on 6/8/25.
//

#include <pybind11/pybind11.h>

#include <chrono>
#include <filesystem>
#include <optional>
#include "foundation/config.hpp"
#include "foundation/logging.hpp"
#include <spdlog/pattern_formatter.h>

namespace py = pybind11;

void init_logger(py::module_& m)
{
    m.def("initialize_logging", &logging::init, "Initialize and enable the logging system");
    m.def("initialize_timer", &logging::initialize_timer, "Initialize the timer for elapsed time logging");
    
    m.def("debug", [](const char* message) { logging::core()->debug(message); }, R"pbdoc(
        Log a message with severity level debug
    )pbdoc", py::arg("message"));

    m.def("info", [](const char* message) { logging::core()->info(message); }, R"pbdoc(
        Log a message with severity level info
    )pbdoc", py::arg("message"));

    m.def("warn", [](const char* message) { logging::core()->warn(message); }, R"pbdoc(
        Log a message with severity level warn
    )pbdoc", py::arg("message"));

    m.def("error", [](const char* message) { logging::core()->error(message); }, R"pbdoc(
        Log a message with severity level error
    )pbdoc", py::arg("message"));
}
