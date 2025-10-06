//
// Created by tobias on 6/8/25.
//

#include <pybind11/pybind11.h>

#include <chrono>
#include <filesystem>
#include <optional>
#include "foundation/config.hpp"
#include <spdlog/pattern_formatter.h>

namespace py = pybind11;

static std::optional<std::chrono::steady_clock::time_point> start_time;

void initialize_timer() {
    start_time = std::chrono::steady_clock::now();
    spdlog::info("Logging timer initialized");
}

class ElapsedTimeFormatter final : public spdlog::custom_flag_formatter {
public:
    void format(const spdlog::details::log_msg &msg, const std::tm &tm, spdlog::memory_buf_t &dest) override {
        if (start_time.has_value()) {
            const auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - *start_time).count() / 1000.0;
            fmt::format_to(std::back_inserter(dest), "{:.3f}s", elapsed);
        } else {
            fmt::format_to(std::back_inserter(dest), "0.000s");
        }
    }

    std::unique_ptr<custom_flag_formatter> clone() const override {
        return spdlog::details::make_unique<ElapsedTimeFormatter>();
    }
};


void initialize_logging() {
    // Ensure the logs directory exists.
    std::filesystem::path log_dir = "logs";
    if (!std::filesystem::exists(log_dir)) {
        std::filesystem::create_directory(log_dir);
    }

    // Create the console and file sinks.
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        (log_dir / "libstp.log").string(),
        5 * 1024 * 1024,  // 5MB max file size
        3                // keep 3 rotated files
    );

    // Create a pattern formatter and register the custom elapsed time flag using '%E'
    auto pattern_formatter = std::make_unique<spdlog::pattern_formatter>();
    pattern_formatter->add_flag<ElapsedTimeFormatter>('E');
    pattern_formatter->set_pattern("[%Y-%m-%d %H:%M:%S] [+%E] [t%t/%n] [%^%l%$]: %v");

    // Apply the custom formatter to both sinks.
    console_sink->set_formatter(pattern_formatter->clone());
    file_sink->set_formatter(pattern_formatter->clone());

    // Create the logger with the custom sinks.
    auto logger = std::make_shared<spdlog::logger>(
        "default", spdlog::sinks_init_list{console_sink, file_sink});

    // Set the default logger and configure its behavior.
    spdlog::set_default_logger(logger);
    logger->set_level(spdlog::level::debug);
    logger->flush_on(spdlog::level::warn);
    spdlog::flush_every(std::chrono::seconds(3));

    logger->info("Logging to directory: {}", std::filesystem::absolute(log_dir).string());
}

void init_logger(py::module_& m)
{
    // Disable logging by default - it will only be enabled when initialize_logging() is called
    spdlog::set_level(spdlog::level::off);

    m.def("initialize_logging", &initialize_logging, "Initialize and enable the logging system");
    m.def("initialize_timer", &initialize_timer, "Initialize the timer for elapsed time logging");
    
    m.def("debug", [](const char* message) { spdlog::debug(message); }, R"pbdoc(
        Log a message with severity level debug
    )pbdoc", py::arg("message"));

    m.def("info", [](const char* message) { spdlog::info(message); }, R"pbdoc(
        Log a message with severity level info
    )pbdoc", py::arg("message"));

    m.def("warn", [](const char* message) { spdlog::warn(message); }, R"pbdoc(
        Log a message with severity level warn
    )pbdoc", py::arg("message"));

    m.def("error", [](const char* message) { spdlog::error(message); }, R"pbdoc(
        Log a message with severity level error
    )pbdoc", py::arg("message"));
}
