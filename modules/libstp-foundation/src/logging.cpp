#include <atomic>
#include <chrono>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
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

        // Console policy: the per-file / per-package / global filter that decides
        // what reaches stdout. This is what set_global_level / set_file_level /
        // set_package_level tune.
        Level global_runtime_level_ = Level::info;

        // File policy: this run's log file captures everything at or above this
        // level regardless of the console policy above. Default TRACE so the log
        // file is a *complete* record (incl. per-tick C++ TRACE telemetry like
        // path-segment progress) even while the console stays at INFO. This is the
        // forensic record we bug-hunt from; the console policy keeps stdout quiet.
        // See is_enabled_for() — a record is admitted to the pipeline if it passes
        // EITHER policy, and the console sink then re-filters on its own policy.
        Level file_runtime_level_ = Level::trace;

        // Weak ref so set_file_log_level() can retune the live file sink (covers
        // the direct logging::core()->debug() bindings that bypass the call-site
        // gate). Must be weak: a strong ref here would outlive spdlog::shutdown(),
        // keeping the file sink alive past the drain so its destructor never
        // flushes/closes — leaving the log file empty.
        std::weak_ptr<spdlog::sinks::sink> file_sink_ref_;

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

        Level min_level(Level a, Level b) {
            return static_cast<int>(a) <= static_cast<int>(b) ? a : b;
        }

        // Effective console level for a source file: exact basename filter, then
        // package-prefix filter, then the global console level. Caller must hold
        // filter_mutex_.
        Level effective_console_level_locked(const char* file) {
            const char* base = detail::basename(file ? file : "");
            auto it = file_filters_.find(base);
            if (it != file_filters_.end()) {
                return it->second;
            }
            if (file && !package_filters_.empty()) {
                for (const auto& [prefix, pkg_level] : package_filters_) {
                    if (strstr(file, prefix.c_str()) != nullptr) {
                        return pkg_level;
                    }
                }
            }
            return global_runtime_level_;
        }

        Level effective_console_level(const char* file) {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            return effective_console_level_locked(file);
        }

        // Intern a source-filename into a process-lifetime pool and return a
        // stable pointer to it. The async backend stores source_loc.filename in
        // the queued record and dereferences it later on the worker thread; a
        // caller-provided pointer (e.g. a transient Python str's UTF-8 buffer)
        // may already be freed by then. node_based unordered_set keeps element
        // addresses stable across inserts/rehashes, and entries are never erased,
        // so the returned pointer is valid for the rest of the process. The set of
        // distinct source files is bounded, so the pool cannot grow without limit.
        const char* intern_source_file(const char* file) {
            if (!file || !*file) {
                return "";
            }
            static std::mutex intern_mutex;
            static std::unordered_set<std::string> pool;
            std::lock_guard<std::mutex> lock(intern_mutex);
            return pool.emplace(file).first->c_str();
        }
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
                if (fs::exists(dir / ".git", ec)
                    || fs::exists(dir / "raccoon.project.yml", ec)
                    || fs::exists(dir / ".raccoon", ec)) {
                    return normalize_separators(dir.string());
                }
                if (dir == dir.root_path()) {
                    break;
                }
            }

            // Fallback: use the current working directory itself so paths
            // outside any detected repo are still shortened to something
            // meaningful instead of being abbreviated from filesystem root.
            return normalize_separators(current.string());
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
        void format(const spdlog::details::log_msg &msg,
                    const std::tm & /*tm*/,
                    spdlog::memory_buf_t &dest) override {
            // Read the source file from the log message itself. The async logger
            // formats on a background worker thread, so a thread-local set by the
            // caller would be unavailable here — the filename rides along in
            // msg.source.filename (populated via spdlog::source_loc in log()).
            const char* file = msg.source.filename;
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

    namespace {
        // Wraps the stdout sink and re-applies the console policy per message so
        // the console stays at its (granular) filter level while the file sink
        // independently receives everything down to file_runtime_level_. Without
        // this, the single shared call-site gate would force the file and console
        // to carry identical content. In async mode only the one backend worker
        // thread drives this, so no extra locking beyond the filter mutex (taken
        // inside effective_console_level) is needed.
        //
        // Note: the console policy is evaluated here at drain time, not at the log
        // call site, so a set_*_level() change applies to records already queued.
        // Filters are set once at startup in practice, so this is benign; the file
        // sink is unaffected — it captures everything down to file_runtime_level_.
        class ConsoleFilterSink final : public spdlog::sinks::sink {
        public:
            explicit ConsoleFilterSink(std::shared_ptr<spdlog::sinks::sink> inner)
                : inner_(std::move(inner)) {}

            void log(const spdlog::details::log_msg& msg) override {
                const char* file = msg.source.filename ? msg.source.filename : "";
                if (msg.level < to_spdlog_level(effective_console_level(file))) {
                    return;
                }
                inner_->log(msg);
            }

            void flush() override { inner_->flush(); }

            void set_pattern(const std::string& pattern) override {
                inner_->set_pattern(pattern);
            }

            void set_formatter(std::unique_ptr<spdlog::formatter> sink_formatter) override {
                inner_->set_formatter(std::move(sink_formatter));
            }

        private:
            std::shared_ptr<spdlog::sinks::sink> inner_;
        };

        // One log file per run, named by wall-clock start time. We do NOT rotate a
        // single growing file — each process gets its own dated file so a run's log
        // is a self-contained artifact you can hand around without slicing a shared
        // file at rotation boundaries.
        constexpr const char* kLogPrefix = "libstp-";
        constexpr const char* kLogSuffix = ".log";

        // How many past runs to keep. Older files are pruned at startup so the log
        // directory doesn't grow without bound.
        constexpr std::size_t kMaxRuns = 25;

        // "libstp-2026-07-01_14-30-00.log" — sortable, filesystem-safe, one per run.
        std::string run_log_filename() {
            const auto now = std::chrono::system_clock::now();
            const std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm_buf{};
#if defined(_WIN32)
            localtime_s(&tm_buf, &t);
#else
            localtime_r(&t, &tm_buf);
#endif
            char stamp[32];
            std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &tm_buf);
            return std::string(kLogPrefix) + stamp + kLogSuffix;
        }

        // Delete the oldest per-run log files, keeping at most `keep` of them
        // (counting the one we're about to create). Files are ordered by name,
        // which sorts chronologically thanks to the zero-padded timestamp.
        void prune_old_runs(const std::filesystem::path& dir, std::size_t keep) {
            namespace fs = std::filesystem;
            std::error_code ec;

            std::vector<std::string> runs;
            for (const auto& entry : fs::directory_iterator(dir, ec)) {
                if (ec) {
                    break;
                }
                if (!entry.is_regular_file(ec)) {
                    continue;
                }
                const std::string name = entry.path().filename().string();
                if (name.rfind(kLogPrefix, 0) == 0
                    && name.size() > std::strlen(kLogSuffix)
                    && name.compare(name.size() - std::strlen(kLogSuffix),
                                    std::strlen(kLogSuffix), kLogSuffix) == 0) {
                    runs.push_back(name);
                }
            }

            std::sort(runs.begin(), runs.end());

            // Leave room for the file this run is about to open.
            const std::size_t budget = keep > 0 ? keep - 1 : 0;
            if (runs.size() <= budget) {
                return;
            }
            const std::size_t to_delete = runs.size() - budget;
            for (std::size_t i = 0; i < to_delete; ++i) {
                fs::remove(dir / runs[i], ec);
            }
        }
    }

    void init() {
        if (logger_initialized) {
            // Logger already initialized, skip
            return;
        }
        logger_initialized = true;

        // Start the relative timer right away.
        initialize_timer();

        // Ensure the logs directory exists.
        std::filesystem::path log_dir{".raccoon/logs"};
        std::filesystem::create_directories(log_dir);

        // Prune old per-run files before opening this run's file so the directory
        // keeps at most kMaxRuns of them (this run included).
        prune_old_runs(log_dir, kMaxRuns);

        // Create console + file sinks. Both keep an spdlog level of trace so they
        // never drop a record on their own; the file's effective floor is enforced
        // by file_runtime_level_ (call-site gate + the level applied below) and the
        // console's floor by ConsoleFilterSink, which re-checks the console policy
        // per message. This split is what lets the file capture all DEBUG output
        // while the console stays quiet.
        auto console_color = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_color->set_level(spdlog::level::trace);

        // One fresh file per run, named by start time (no rotation of a shared
        // file). Old runs are pruned above; see run_log_filename()/prune_old_runs().
        const std::string log_path = (log_dir / run_log_filename()).string();
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path);
        file_sink->set_level(to_spdlog_level(file_runtime_level_));
        file_sink_ref_ = file_sink;

        // Pattern formatter with custom flags: '%E' for elapsed time, '%*' for source file.
        auto pattern_formatter = std::make_unique<spdlog::pattern_formatter>();
        pattern_formatter->add_flag<ElapsedTimeFormatter>('E');
        pattern_formatter->add_flag<SourceFileFormatter>('*');
        pattern_formatter->set_pattern(
            "%Y-%m-%d %H:%M:%S | %E | %^%-8l%$ | %* | %v"
        );

        console_color->set_formatter(pattern_formatter->clone());
        file_sink->set_formatter(std::move(pattern_formatter));

        // Console output is filtered by the console policy; the file sink is not.
        auto console_sink = std::make_shared<ConsoleFilterSink>(console_color);

        // Single core ASYNC logger, used as default for the whole app.
        //
        // The actual sink writes — stdout (a pipe / journald on the Pi) and the
        // per-run file on the SD card — must NOT run on the caller. Steps log
        // their signature via self.info() *on the asyncio mission loop*; a
        // synchronous write that blocks on an SD write-back
        // would stall the entire mission loop (the same hot-path stall that hit
        // the stm32-data-reader). A background worker drains a bounded queue;
        // overrun_oldest drops the oldest line when the queue is full instead of
        // blocking the caller — losing a log line beats stalling the robot.
        spdlog::init_thread_pool(8192, 1);
        auto logger = std::make_shared<spdlog::async_logger>(
            "core", spdlog::sinks_init_list{console_sink, file_sink},
            spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest
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

    bool is_enabled(Level level) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return false;
        }
        // Logging must be explicitly initialized (via initialize_logging() /
        // GenericRobot). Importing the library alone must not spawn log dirs,
        // sinks, or flush threads — that would also re-init spdlog during
        // C++ static teardown when atexit-driven motor disarms fire.
        if (!logger_initialized) {
            return false;
        }
        // Admit the record if it passes EITHER the console policy or the file
        // policy — i.e. the more permissive of the two floors. The per-sink
        // filters then decide which sinks actually emit it.
        Level floor;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            floor = min_level(global_runtime_level_, file_runtime_level_);
        }
        return static_cast<int>(level) >= static_cast<int>(floor);
    }

    bool is_enabled_for(Level level, const char* file) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return false;
        }
        if (!logger_initialized) {
            return false;
        }

        // Admit if the record passes the file policy (flat floor, default DEBUG)
        // or the console policy for this file (basename → package → global). The
        // ConsoleFilterSink re-applies the console policy so the console is not
        // spammed by records admitted only for the file.
        Level floor;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            floor = min_level(effective_console_level_locked(file), file_runtime_level_);
        }
        return static_cast<int>(level) >= static_cast<int>(floor);
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
        // Note: file_runtime_level_ is intentionally left untouched — the file's
        // full-debug capture is a separate policy from the console filters and is
        // reset only via set_file_log_level().
    }

    void set_file_log_level(Level level) {
        std::shared_ptr<spdlog::sinks::sink> sink;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            file_runtime_level_ = level;
            sink = file_sink_ref_.lock();
        }
        // Keep the live sink in sync so even the direct logging::core()->debug()
        // bindings (which bypass the call-site gate) honour the file floor.
        if (sink) {
            sink->set_level(to_spdlog_level(level));
        }
    }

    Level get_file_log_level() {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        return file_runtime_level_;
    }

    void log(Level level, std::string_view message) {
        if (logger_shutdown.load(std::memory_order_relaxed)) {
            return;
        }
        if (!logger_initialized) {
            return;
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
            return;
        }
        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return;
        }
        // Carry the source file in the log message (source_loc) rather than a
        // thread-local: the async backend formats on a worker thread, so the
        // SourceFileFormatter must read it from the message. The pointer must
        // outlive the queued record, so intern it into a process-lifetime pool —
        // a caller-provided pointer (e.g. a transient Python str) may otherwise be
        // freed before the worker formats it.
        logger->log(spdlog::source_loc{intern_source_file(source_file), 0, ""},
                    to_spdlog_level(level), message);
    }

    void shutdown() {
        logger_shutdown.store(true, std::memory_order_relaxed);
        spdlog::shutdown();
        logger_initialized = false;
    }

} // namespace logging
