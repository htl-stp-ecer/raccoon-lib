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
    m.def("shutdown_logging", &logging::shutdown,
          "Flush and tear down the logger so that no further log calls touch spdlog");
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

    m.def("_log", [](logging::Level level, const char* filepath, int line,
                     const char* func, const char* message) {
        logging::log(level, filepath, line, func, message);
    }, R"pbdoc(
        Log a message annotating its Python source location (internal use).

        The file / line / func land as discrete fields in the JSONL record.
        logging::log() drops the call if logging is not live; there is no runtime
        level filtering (what is compiled in is captured).

        Args:
            level: The log level
            filepath: The Python source file path
            line: The Python source line number
            func: The originating function (qualified as "Class.method" when logged
                through a ClassNameLogger)
            message: The log message
    )pbdoc", py::arg("level"), py::arg("filepath"), py::arg("line"),
       py::arg("func"), py::arg("message"));
}
