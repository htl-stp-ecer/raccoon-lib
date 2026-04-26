#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/pattern_formatter.h>
#include "foundation/logging.hpp"

#include <iostream>

namespace logging {

    namespace {
        std::optional<std::chrono::steady_clock::time_point> start_time;
        bool logger_initialized = false;
        std::atomic<bool> logger_shutdown{false};

        // Runtime filtering state
        std::mutex filter_mutex_;
        std::unordered_map<std::string, Level> file_filters_;
        std::vector<std::pair<std::string, Level>> package_filters_;
        Level global_runtime_level_ = Level::info;

        // Thread-local source filename for the custom formatter
        thread_local const char* current_source_file_ = "";
    }

    /// Call this if you ever want to reset the relative timer at runtime.
    void initialize_timer() {
        start_time = std::chrono::steady_clock::now();
    }

    class ElapsedTimeFormatter final : public spdlog::custom_flag_formatter {
    public:
        void format(const spdlog::details::log_msg &msg,
                    const std::tm & /*tm*/,
                    spdlog::memory_buf_t &dest) override {
            if (start_time.has_value()) {
                const auto now = std::chrono::steady_clock::now();
                const double elapsed =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - *start_time).count()
                    / 1000.0;
                fmt::format_to(std::back_inserter(dest), "{:>9.3f}s", elapsed);
            } else {
                fmt::format_to(std::back_inserter(dest), "    0.000s");
            }
        }

        std::unique_ptr<custom_flag_formatter> clone() const override {
            return spdlog::details::make_unique<ElapsedTimeFormatter>();
        }
    };

    class SourceFileFormatter final : public spdlog::custom_flag_formatter {
        static std::string normalize_separators(std::string path) {
            for (char& c : path) {
                if (c == '\\') {
                    c = '/';
                }
            }
            return path;
        }

        static std::optional<std::string> detect_repo_root() {
            namespace fs = std::filesystem;
            std::error_code ec;
            auto current = fs::current_path(ec);
            if (ec) {
                return std::nullopt;
            }

            current = current.lexically_normal();
            for (auto dir = current; !dir.empty(); dir = dir.parent_path()) {
                if (fs::exists(dir / ".git", ec)) {
                    return normalize_separators(dir.string());
                }
                if (dir == dir.root_path()) {
                    break;
                }
            }

            return std::nullopt;
        }

        static std::string repo_relative_path(const char* path) {
            std::string normalized = normalize_separators(path ? path : "");
            if (normalized.empty() || normalized[0] != '/') {
                return normalized;
            }

            static const std::optional<std::string> repo_root = detect_repo_root();
            if (!repo_root || repo_root->empty()) {
                return normalized;
            }

            const std::string prefix = *repo_root + "/";
            if (normalized.rfind(prefix, 0) == 0) {
                return normalized.substr(prefix.size());
            }

            if (normalized == *repo_root) {
                return ".";
            }

            return normalized;
        }

        static bool is_sep(char c) {
            return c == '/' || c == '\\';
        }

        static void format_abbreviated_path(const char* path, spdlog::memory_buf_t &dest) {
            const char* last_sep = nullptr;
            for (const char* c = path; *c; ++c) {
                if (is_sep(*c)) {
                    last_sep = c;
                }
            }

            if (!last_sep) {
                fmt::format_to(std::back_inserter(dest), "{}", path);
                return;
            }

            const char* file = last_sep + 1;
            const char* start = path;
            for (const char* c = path; c < file; ++c) {
                if (is_sep(*c)) {
                    if (c > start && !(c - start == 1 && start[0] == '.')) {
                        fmt::format_to(std::back_inserter(dest), "{}.", *start);
                    }
                    start = c + 1;
                }
            }

            if (*file) {
                fmt::format_to(std::back_inserter(dest), "{}", file);
            } else {
                fmt::format_to(std::back_inserter(dest), "{}", detail::basename(path));
            }
        }

        static void format_package(const char* path, spdlog::memory_buf_t &dest) {
            const std::string display_path = repo_relative_path(path);
            const char* display = display_path.c_str();

            // Python style: .../libstp/<pkg>/<pkg>/<file>.py → p.p.file.py (Spring-style)
            const char* p = strstr(display, "libstp/");
            if (!p) {
                p = strstr(display, "libstp\\");
            }
            if (p) {
                p += 7;
                // Find last separator to know which component is the filename.
                const char* last_sep = nullptr;
                for (const char* c = p; *c; ++c) {
                    if (is_sep(*c)) {
                        last_sep = c;
                    }
                }

                const char* start = p;
                bool first = true;
                for (const char* c = p; ; ++c) {
                    if (is_sep(*c) || *c == '\0') {
                        if (c > start) {
                            if (!first) fmt::format_to(std::back_inserter(dest), ".");
                            first = false;
                            if (last_sep && c <= last_sep) {
                                // Shorten intermediate components to first char
                                fmt::format_to(std::back_inserter(dest), "{}", *start);
                            } else {
                                // Last component: keep full name
                                fmt::format_to(std::back_inserter(dest), "{}",
                                               std::string_view(start, c - start));
                            }
                        }
                        if (*c == '\0') break;
                        start = c + 1;
                    }
                }
                return;
            }

            // C++ style: .../libstp-<module>/src/<file>.cpp → m.file.cpp (Spring-style)
            p = strstr(display, "libstp-");
            if (p && p[7] && !is_sep(p[7])) {
                const char* base = detail::basename(display);
                fmt::format_to(std::back_inserter(dest), "{}.{}", p[7], base);
                return;
            }

            // Generic fallback: abbreviate all directories and keep full filename
            // (e.g., /a/b/c/file.py -> a.b.c.file.py).
            format_abbreviated_path(display, dest);
        }

    public:
        void format(const spdlog::details::log_msg & /*msg*/,
                    const std::tm & /*tm*/,
                    spdlog::memory_buf_t &dest) override {
            const char* file = current_source_file_;
            auto start = dest.size();
            if (file && *file) {
                format_package(file, dest);
            }
            // Pad to fixed width (30 chars, left-aligned)
            constexpr char spaces[] = "                              ";
            auto written = dest.size() - start;
            if (written < 30) {
                auto pad = 30 - written;
                dest.append(spaces, spaces + pad);
            }
        }

        std::unique_ptr<custom_flag_formatter> clone() const override {
            return spdlog::details::make_unique<SourceFileFormatter>();
        }
    };

    void init() {
        if (logger_initialized) {
            // Logger already initialized, skip
            return;
        }
        logger_initialized = true;

        // Start the relative timer right away.
        initialize_timer();

        // Ensure the logs directory exists.
        std::filesystem::path log_dir{"logs"};
        if (!std::filesystem::exists(log_dir)) {
            std::filesystem::create_directory(log_dir);
        }

        // Create console + file sinks.
        // Set to trace level to allow all messages; filtering is done at runtime
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::trace);

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            (log_dir / "libstp.log").string(),
            5 * 1024 * 1024,  // 5MB max file size
            3                 // keep 3 rotated files
        );
        file_sink->set_level(spdlog::level::trace);

        // Pattern formatter with custom flags: '%E' for elapsed time, '%*' for source file.
        auto pattern_formatter = std::make_unique<spdlog::pattern_formatter>();
        pattern_formatter->add_flag<ElapsedTimeFormatter>('E');
        pattern_formatter->add_flag<SourceFileFormatter>('*');
        pattern_formatter->set_pattern(
            "%Y-%m-%d %H:%M:%S | %E | %^%-8l%$ | %* | %v"
        );

        console_sink->set_formatter(pattern_formatter->clone());
        file_sink->set_formatter(std::move(pattern_formatter));

        // Single core logger, used as default for the whole app.
        auto logger = std::make_shared<spdlog::logger>(
            "core", spdlog::sinks_init_list{console_sink, file_sink}
        );

        logger->set_level(spdlog::level::trace);
        logger->flush_on(spdlog::level::warn);

        spdlog::set_default_logger(logger); // SPDLOG_* macros use this
        spdlog::flush_every(std::chrono::seconds(3));

        logger->info("Logging to directory: {}", std::filesystem::absolute(log_dir).string());
    }

    std::shared_ptr<spdlog::logger> core() {
        auto logger = spdlog::get("core");
        if (!logger) {
            throw std::runtime_error("logging::init() must be called before using logging::core()");
        }
        return logger;
    }

    namespace {
        spdlog::level::level_enum to_spdlog_level(Level level) {
            switch (level) {
                case Level::trace: return spdlog::level::trace;
                case Level::debug: return spdlog::level::debug;
                case Level::info: return spdlog::level::info;
                case Level::warn: return spdlog::level::warn;
                case Level::error: return spdlog::level::err;
                case Level::critical: return spdlog::level::critical;
                case Level::off: return spdlog::level::off;
            }
            return spdlog::level::info;
        }
    }

    bool is_enabled(Level level) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return false;
        }
        // Auto-initialize if not done yet
        if (!logger_initialized) {
            init();
        }
        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return false;
        }
        return logger->should_log(to_spdlog_level(level));
    }

    bool is_enabled_for(Level level, const char* file) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return false;
        }
        // Auto-initialize if not done yet
        if (!logger_initialized) {
            init();
        }

        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return false;
        }

        // Determine effective level: basename exact match → package prefix → global
        Level effective_level;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);

            // 1. Exact basename match (fast path)
            const char* base = detail::basename(file);
            auto it = file_filters_.find(base);
            if (it != file_filters_.end()) {
                effective_level = it->second;
            }
            // 2. Package prefix match against the full path
            else if (!package_filters_.empty()) {
                bool matched = false;
                for (const auto& [prefix, pkg_level] : package_filters_) {
                    if (strstr(file, prefix.c_str()) != nullptr) {
                        effective_level = pkg_level;
                        matched = true;
                        break;
                    }
                }
                if (!matched) {
                    effective_level = global_runtime_level_;
                }
            } else {
                effective_level = global_runtime_level_;
            }
        }

        return static_cast<int>(level) >= static_cast<int>(effective_level);
    }

    void set_global_level(Level level) {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        global_runtime_level_ = level;
    }

    void set_file_level(const std::string& filename, Level level) {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        file_filters_[filename] = level;
    }

    void clear_file_level(const std::string& filename) {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        file_filters_.erase(filename);
    }

    void set_package_level(const std::string& package, Level level) {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        // Update existing entry if present
        for (auto& [prefix, lvl] : package_filters_) {
            if (prefix == package) {
                lvl = level;
                return;
            }
        }
        package_filters_.emplace_back(package, level);
    }

    void clear_package_level(const std::string& package) {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        package_filters_.erase(
            std::remove_if(package_filters_.begin(), package_filters_.end(),
                           [&](const auto& entry) { return entry.first == package; }),
            package_filters_.end());
    }

    void clear_filters() {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        file_filters_.clear();
        package_filters_.clear();
        global_runtime_level_ = Level::info;
    }

    void log(Level level, std::string_view message) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return;
        }
        // Auto-initialize if not done yet
        if (!logger_initialized) {
            init();
        }
        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return;
        }
        logger->log(to_spdlog_level(level), message);
    }

    void log(Level level, const char* source_file, std::string_view message) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return;
        }
        if (!logger_initialized) {
            init();
        }
        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return;
        }
        current_source_file_ = source_file ? source_file : "";
        logger->log(to_spdlog_level(level), message);
        current_source_file_ = "";
    }

    void shutdown() {
        logger_shutdown.store(true, std::memory_order_relaxed);
        spdlog::shutdown();
        logger_initialized = false;
    }

} // namespace logging
