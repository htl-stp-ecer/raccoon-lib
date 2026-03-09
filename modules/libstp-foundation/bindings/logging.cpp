//
// Created by tobias on 6/8/25.
//

#include <pybind11/pybind11.h>

#include <chrono>
#include <filesystem>
#include <optional>
#include "foundation/config.hpp"
#include "foundation/logging.hpp"

#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/pattern_formatter.h>

namespace py = pybind11;

void init_logger(py::module_& m)
{
    // Expose the Level enum (don't export_values to avoid conflict with debug/info/warn/error functions)
    py::enum_<logging::Level>(m, "Level")
        .value("trace", logging::Level::trace)
        .value("debug", logging::Level::debug)
        .value("info", logging::Level::info)
        .value("warn", logging::Level::warn)
        .value("error", logging::Level::error)
        .value("critical", logging::Level::critical)
        .value("off", logging::Level::off);

    m.def("initialize_logging", &logging::init, "Initialize and enable the logging system");
    m.def("initialize_timer", &logging::initialize_timer, "Initialize the timer for elapsed time logging");

    // Runtime log level filtering API
    m.def("set_global_level", &logging::set_global_level, R"pbdoc(
        Set the global runtime log level. Messages below this level are filtered.

        Args:
            level: The minimum log level to display (e.g., Level.trace, Level.debug, Level.info)
    )pbdoc", py::arg("level"));

    m.def("set_file_level", &logging::set_file_level, R"pbdoc(
        Set the log level for a specific source file (by basename).

        Args:
            filename: The source file basename (e.g., "fused_odometry.cpp")
            level: The minimum log level for this file
    )pbdoc", py::arg("filename"), py::arg("level"));

    m.def("clear_file_level", &logging::clear_file_level, R"pbdoc(
        Remove the log level filter for a specific source file.

        Args:
            filename: The source file basename to clear
    )pbdoc", py::arg("filename"));

    m.def("clear_filters", &logging::clear_filters, R"pbdoc(
        Clear all file-specific filters and reset global level to INFO.
    )pbdoc");

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

    m.def("set_package_level", &logging::set_package_level, R"pbdoc(
        Set the log level for all source files whose path contains the given substring.

        Package filters are checked after file-specific filters (set_file_level).
        Use path fragments like "libstp-motion" (C++ module) or "libstp/step/motion"
        (Python package) to match groups of files.

        Args:
            package: A substring to match against the full source file path
            level: The minimum log level for matching files
    )pbdoc", py::arg("package"), py::arg("level"));

    m.def("clear_package_level", &logging::clear_package_level, R"pbdoc(
        Remove the log level filter for a package substring.

        Args:
            package: The package substring to clear
    )pbdoc", py::arg("package"));

    m.def("_log_filtered", [](logging::Level level, const char* filepath, const char* message) {
        if (logging::is_enabled_for(level, filepath)) {
            logging::log(level, filepath, message);
        }
    }, R"pbdoc(
        Log a message with file-based filtering (internal use).

        Checks is_enabled_for(level, filepath) before logging, supporting both
        per-file filtering (set_file_level) and package filtering (set_package_level).

        Args:
            level: The log level
            filepath: The full source file path (used for filtering and display)
            message: The log message
    )pbdoc", py::arg("level"), py::arg("filepath"), py::arg("message"));
}
