//
// Created by tobias on 11/27/25.
//

#pragma once

#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <spdlog/fmt/bundled/format.h>

// Compile-time level values for the custom macros.
#define LIBSTP_LOG_LEVEL_TRACE 0
#define LIBSTP_LOG_LEVEL_DEBUG 1
#define LIBSTP_LOG_LEVEL_INFO 2
#define LIBSTP_LOG_LEVEL_WARN 3
#define LIBSTP_LOG_LEVEL_ERROR 4
#define LIBSTP_LOG_LEVEL_CRITICAL 5
#define LIBSTP_LOG_LEVEL_OFF 6

#ifndef FOUNDATION_LOG_ACTIVE_LEVEL
#define FOUNDATION_LOG_ACTIVE_LEVEL LIBSTP_LOG_LEVEL_INFO
#endif

namespace spdlog { class logger; }

namespace logging {

    /// Runtime severity used by the foundation logger and its Python bindings.
    enum class Level : int {
        trace = LIBSTP_LOG_LEVEL_TRACE,
        debug = LIBSTP_LOG_LEVEL_DEBUG,
        info = LIBSTP_LOG_LEVEL_INFO,
        warn = LIBSTP_LOG_LEVEL_WARN,
        error = LIBSTP_LOG_LEVEL_ERROR,
        critical = LIBSTP_LOG_LEVEL_CRITICAL,
        off = LIBSTP_LOG_LEVEL_OFF,
    };

    namespace detail {
        constexpr int level_value(Level level) {
            return static_cast<int>(level);
        }

        // Extract basename from full path (e.g., "/path/to/file.cpp" -> "file.cpp")
        constexpr const char* basename(const char* path) {
            const char* file = path;
            while (*path) {
                if (*path == '/' || *path == '\\') {
                    file = path + 1;
                }
                ++path;
            }
            return file;
        }
    }

    /// Reset the relative elapsed-time clock used by the log formatter.
    void initialize_timer();

    /// Initialize the shared logger, sinks, and formatter. Safe to call more than once.
    void init();

    /// Runtime log level filtering API.
    void set_global_level(Level level);
    void set_file_level(const std::string& filename, Level level);
    void clear_file_level(const std::string& filename);
    void set_package_level(const std::string& package, Level level);
    void clear_package_level(const std::string& package);
    void clear_filters();

    /// Query whether a log level is enabled by the current runtime filter state.
    bool is_enabled(Level level);

    /// Query whether a log level is enabled for a source file.
    /// Accepts either a basename ("drive.cpp") or a full/repo-relative path.
    /// Checks exact basename match first, then package prefix filters.
    bool is_enabled_for(Level level, const char* file);

    /// Log a preformatted message without source-file context.
    void log(Level level, std::string_view message);

    /// Log a preformatted message while annotating the originating source file.
    void log(Level level, const char* source_file, std::string_view message);

    // Format-aware logging helper that keeps formatting outside of the logging backend.
    template <typename... Args>
    inline void logf(Level level, std::string_view fmt_str, Args&&... args) {
        if (!is_enabled(level)) {
            return;
        }
        if constexpr (sizeof...(Args) == 0) {
            log(level, fmt_str);
        } else {
            auto args_tuple = std::make_tuple(std::forward<Args>(args)...);
            auto message = std::apply(
                [&](auto&... unpacked) {
                    return fmt::vformat(fmt::string_view(fmt_str.data(), fmt_str.size()),
                                        fmt::make_format_args(unpacked...));
                },
                args_tuple);
            log(level, message);
        }
    }

    // Format-aware logging helper with file-based filtering
    template <typename... Args>
    inline void logf_file(Level level, const char* filter_name, const char* display_path,
                          std::string_view fmt_str, Args&&... args) {
        if (!is_enabled_for(level, filter_name)) {
            return;
        }
        if constexpr (sizeof...(Args) == 0) {
            log(level, display_path, fmt_str);
        } else {
            auto args_tuple = std::make_tuple(std::forward<Args>(args)...);
            auto message = std::apply(
                [&](auto&... unpacked) {
                    return fmt::vformat(fmt::string_view(fmt_str.data(), fmt_str.size()),
                                        fmt::make_format_args(unpacked...));
                },
                args_tuple);
            log(level, display_path, message);
        }
    }

    /// Access the shared spdlog logger after initialization.
    std::shared_ptr<spdlog::logger> core();
}

#define LIBSTP_LOG_CALL(level_enum, fmt, ...)                                                      \
    do {                                                                                           \
        if constexpr (::logging::detail::level_value(level_enum) >= FOUNDATION_LOG_ACTIVE_LEVEL) { \
            ::logging::logf_file(level_enum, __FILE__,                                            \
                                 __FILE__,                                                         \
                                 fmt __VA_OPT__(, __VA_ARGS__));                                   \
        }                                                                                          \
    } while (0)

#define LIBSTP_LOG_TRACE(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::trace, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_DEBUG(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::debug, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_INFO(fmt, ...)     LIBSTP_LOG_CALL(::logging::Level::info, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_WARN(fmt, ...)     LIBSTP_LOG_CALL(::logging::Level::warn, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_ERROR(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::error, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_CRITICAL(fmt, ...) LIBSTP_LOG_CALL(::logging::Level::critical, fmt __VA_OPT__(, __VA_ARGS__))
