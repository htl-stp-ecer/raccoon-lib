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
    }

    void initialize_timer();
    void init();

    bool is_enabled(Level level);
    void log(Level level, std::string_view message);

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

    std::shared_ptr<spdlog::logger> core();
}

#define LIBSTP_LOG_CALL(level_enum, fmt, ...)                                                      \
    do {                                                                                           \
        if constexpr (::logging::detail::level_value(level_enum) >= FOUNDATION_LOG_ACTIVE_LEVEL) { \
            ::logging::logf(level_enum, fmt __VA_OPT__(, __VA_ARGS__));                            \
        }                                                                                          \
    } while (0)

#define LIBSTP_LOG_TRACE(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::trace, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_DEBUG(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::debug, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_INFO(fmt, ...)     LIBSTP_LOG_CALL(::logging::Level::info, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_WARN(fmt, ...)     LIBSTP_LOG_CALL(::logging::Level::warn, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_ERROR(fmt, ...)    LIBSTP_LOG_CALL(::logging::Level::error, fmt __VA_OPT__(, __VA_ARGS__))
#define LIBSTP_LOG_CRITICAL(fmt, ...) LIBSTP_LOG_CALL(::logging::Level::critical, fmt __VA_OPT__(, __VA_ARGS__))
