#include "transport_core/iox2_log_bridge.hpp"

#include "foundation/logging.hpp"

#include <iox2/log.hpp>
#include <iox2/log_level.hpp>

#include <atomic>
#include <cstdio>
#include <string>

namespace libstp::transport_core {

namespace {

// Map iceoryx2's LogLevel onto the foundation Level enum. Fatal collapses
// onto critical — spdlog has no separate fatal tier.
::logging::Level to_foundation_level(iox2::LogLevel lvl) {
    switch (lvl) {
        case iox2::LogLevel::Trace: return ::logging::Level::trace;
        case iox2::LogLevel::Debug: return ::logging::Level::debug;
        case iox2::LogLevel::Info:  return ::logging::Level::info;
        case iox2::LogLevel::Warn:  return ::logging::Level::warn;
        case iox2::LogLevel::Error: return ::logging::Level::error;
        case iox2::LogLevel::Fatal: return ::logging::Level::critical;
    }
    return ::logging::Level::info;
}

const char* level_tag(iox2::LogLevel lvl) {
    switch (lvl) {
        case iox2::LogLevel::Trace: return "trace";
        case iox2::LogLevel::Debug: return "debug";
        case iox2::LogLevel::Info:  return "info";
        case iox2::LogLevel::Warn:  return "warning";
        case iox2::LogLevel::Error: return "error";
        case iox2::LogLevel::Fatal: return "critical";
    }
    return "info";
}

class Iox2SpdlogBridge final : public iox2::Log {
public:
    void log(iox2::LogLevel lvl, const char* origin, const char* message) override {
        // Drop the giant Debug-repr origin strings that iceoryx2 emits as
        // "context" — they're 2-3 KB struct dumps. The actual message
        // already carries the useful information.
        (void)origin;

        const auto level = to_foundation_level(lvl);
        const std::string_view body = message ? message : "";

        // Filter at our layer: anything below Warn from iox2 is noise.
        // We do NOT call iox2::set_log_level — touching iox2's global log
        // state too early appears to disturb NodeBuilder initialization
        // (NodeCreationFailure panics). Filtering here keeps iox2 in its
        // default INFO mode but drops the chatter before it hits spdlog.
        if (lvl < iox2::LogLevel::Warn) {
            return;
        }

        if (::logging::is_enabled(level)) {
            // spdlog ready → use it. Pass "iox2" as the source-file tag so
            // the formatter renders it in the source column.
            ::logging::log(level, "iox2", body);
            return;
        }

        // Early-startup fallback: spdlog not yet initialized. Write to
        // stderr in a compact form so the line is preserved but doesn't
        // look like the native iox2 multi-line struct dump.
        std::fprintf(stderr, "[iox2 %s] %s\n", level_tag(lvl),
                     message ? message : "");
    }
};

std::atomic<bool> g_installed{false};

} // namespace

void install_iox2_log_bridge() {
    bool expected = false;
    if (!g_installed.compare_exchange_strong(expected, true)) {
        return;
    }

    // The bridge instance must outlive every subsequent iox2 call. Static
    // storage at function scope keeps it alive until process exit and
    // initializes it on first use (after CRT static-init storms).
    //
    // We intentionally do NOT call iox2::set_log_level here. An earlier
    // attempt to clamp the iox2 global level to Warn via
    // set_log_level_from_env_or(Warn) caused subsequent NodeBuilder::create
    // calls to fail with NodeCreationFailure → process panic. The Warn
    // threshold is enforced inside Iox2SpdlogBridge::log() instead, which
    // keeps iox2's own initialization paths untouched.
    static Iox2SpdlogBridge bridge;
    iox2::set_logger(bridge);
}

} // namespace libstp::transport_core
