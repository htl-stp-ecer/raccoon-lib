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

// Compile-time floor for the LIBSTP_LOG_* macros. DEBUG by default so C++ debug
// statements are compiled in and can reach the per-run log file; TRACE stays
// behind the LIBSTP_TRACE_LOGGING build flag (see CMakeLists.txt) to avoid the
// cost of the highest-rate trace sites in normal builds.
#ifndef FOUNDATION_LOG_ACTIVE_LEVEL
#define FOUNDATION_LOG_ACTIVE_LEVEL LIBSTP_LOG_LEVEL_DEBUG
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

    /// Query whether logging is live (initialized and not shut down). There is no
    /// runtime level filtering: what is compiled in (the FOUNDATION_LOG_ACTIVE_LEVEL
    /// gate at the call site) is what the JSONL file captures. The console shows
    /// warn/error only, enforced natively by the console sink's level.
    bool is_enabled(Level level);

    /// Log a preformatted message without source-location context.
    void log(Level level, std::string_view message);

    /// Log a preformatted message while annotating the originating source location
    /// (file / line / function). These land as discrete fields in the JSONL record.
    void log(Level level, const char* source_file, int line, const char* func,
             std::string_view message);

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

    // Format-aware logging helper that annotates the originating source location.
    template <typename... Args>
    inline void logf_loc(Level level, const char* file, int line, const char* func,
                         std::string_view fmt_str, Args&&... args) {
        if (!is_enabled(level)) {
            return;
        }
        if constexpr (sizeof...(Args) == 0) {
            log(level, file, line, func, fmt_str);
        } else {
            auto args_tuple = std::make_tuple(std::forward<Args>(args)...);
            auto message = std::apply(
                [&](auto&... unpacked) {
                    return fmt::vformat(fmt::string_view(fmt_str.data(), fmt_str.size()),
                                        fmt::make_format_args(unpacked...));
                },
                args_tuple);
            log(level, file, line, func, message);
        }
    }

    /// Flush and tear down the logger so that no further log calls touch spdlog.
    /// Must be called before C++ static destruction begins (e.g. from a Python
    /// atexit handler) to avoid use-after-free in cross-library static teardown.
    void shutdown();

    /// Access the shared spdlog logger after initialization.
    std::shared_ptr<spdlog::logger> core();
}

#define LIBSTP_LOG_CALL(level_enum, fmt, ...)                                                      \
    do {                                                                                           \
        if constexpr (::logging::detail::level_value(level_enum) >= FOUNDATION_LOG_ACTIVE_LEVEL) { \
            ::logging::logf_loc(level_enum, __FILE__, __LINE__,                                     \
                                static_cast<const char*>(__FUNCTION__),                             \
                                fmt __VA_OPT__(, __VA_ARGS__));                                      \
        }                                                                                          \
    } while (0)

#define LIBSTP_LOG_TRACE(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::trace, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_DEBUG(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::debug, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_INFO(fmt, ...)     LIBSTP_LOG_CALL(::logging::Level::info, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_WARN(fmt, ...)     LIBSTP_LOG_CALL(::logging::Level::warn, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_ERROR(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::error, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_CRITICAL(fmt, ...) LIBSTP_LOG_CALL(::logging::Level::critical, fmt __VA_OPT__(, __VA_ARGS__))
