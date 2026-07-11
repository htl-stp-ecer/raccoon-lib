#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <algorithm>
#include <unordered_set>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/pattern_formatter.h>
#include "foundation/logging.hpp"

namespace logging {

    namespace {
        // Two anchors for the same t=0. The wall-clock 't' field is a display
        // timestamp derived from the system clock (so it reads as real local
        // time). The 'elapsed' field is a DURATION, so it is measured against
        // the monotonic steady clock: a system-clock step (NTP sync on a Pi with
        // no RTC, manual set) must never make 'elapsed' jump backwards or forwards.
        std::optional<std::chrono::steady_clock::time_point> start_time_steady;
        std::optional<std::chrono::system_clock::time_point> start_time_system;
        bool logger_initialized = false;
        std::atomic<bool> logger_shutdown{false};

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

        // Intern a source-filename / function-name into a process-lifetime pool
        // and return a stable pointer to it. The async backend stores the
        // source_loc pointers in the queued record and dereferences them later on
        // the worker thread; a caller-provided pointer (e.g. a transient Python
        // str's UTF-8 buffer) may already be freed by then. A node-based
        // unordered_set keeps element addresses stable across inserts/rehashes and
        // entries are never erased, so the returned pointer is valid for the rest
        // of the process. The set of distinct strings is bounded, so the pool
        // cannot grow without limit.
        const char* intern_string(const char* s) {
            if (!s || !*s) {
                return "";
            }
            static std::mutex intern_mutex;
            static std::unordered_set<std::string> pool;
            std::lock_guard<std::mutex> lock(intern_mutex);
            return pool.emplace(s).first->c_str();
        }

        long process_id() {
#if defined(_WIN32)
            static const long pid = static_cast<long>(_getpid());
#else
            static const long pid = static_cast<long>(::getpid());
#endif
            return pid;
        }
    }

    /// Call this if you ever want to reset the relative timer at runtime.
    void initialize_timer() {
        start_time_steady = std::chrono::steady_clock::now();
        start_time_system = std::chrono::system_clock::now();
    }

    namespace {
        // One JSON object per line. The file sink is the complete forensic record
        // of a run; we dump every field discretely (no path shortening, no fixed
        // column layout) so post-hoc tooling on the PC can filter/format instead of
        // the robot spending cycles pretty-printing a log nobody reads live.
        class JsonLineFormatter final : public spdlog::formatter {
        public:
            void format(const spdlog::details::log_msg& msg,
                        spdlog::memory_buf_t& dest) override {
                auto out = std::back_inserter(dest);

                // Wall-clock timestamp (local time, millisecond precision).
                const std::time_t tt = std::chrono::system_clock::to_time_t(msg.time);
                std::tm tm_buf{};
#if defined(_WIN32)
                localtime_s(&tm_buf, &tt);
#else
                localtime_r(&tt, &tm_buf);
#endif
                char stamp[32];
                std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tm_buf);
                const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    msg.time.time_since_epoch()).count() % 1000;

                // Seconds since initialize_timer(), from the monotonic clock so a
                // wall-clock step never corrupts it. Sampled here on the async
                // worker thread (i.e. at drain time, not log-call time); the queue
                // is a single-worker FIFO so 'elapsed' stays monotonically ordered,
                // and the residence lag is sub-millisecond — a fair trade for
                // immunity to clock jumps. 't' above keeps call-time system time.
                double elapsed = 0.0;
                if (start_time_steady.has_value()) {
                    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - *start_time_steady).count()
                              / 1000.0;
                }

                const auto seq = seq_counter_.fetch_add(1, std::memory_order_relaxed);
                const auto level_sv = spdlog::level::to_string_view(msg.level);

                fmt::format_to(out,
                    R"({{"t":"{}.{:03d}","elapsed":{:.3f},"seq":{},"level":")",
                    stamp, ms, elapsed, seq);
                append_escaped(dest, std::string_view(level_sv.data(), level_sv.size()));
                fmt::format_to(out, R"(","logger":")");
                append_escaped(dest,
                    std::string_view(msg.logger_name.data(), msg.logger_name.size()));
                fmt::format_to(out, R"(","thread":{},"pid":{},"file":")",
                    msg.thread_id, process_id());
                if (msg.source.filename) {
                    append_escaped(dest, msg.source.filename);
                }
                fmt::format_to(out, R"(","line":{},"func":")", msg.source.line);
                if (msg.source.funcname) {
                    append_escaped(dest, msg.source.funcname);
                }
                fmt::format_to(out, R"(","msg":")");
                append_escaped(dest,
                    std::string_view(msg.payload.data(), msg.payload.size()));
                fmt::format_to(out, "\"}}");
                dest.push_back('\n');
            }

            std::unique_ptr<spdlog::formatter> clone() const override {
                return spdlog::details::make_unique<JsonLineFormatter>();
            }

        private:
            // Drain order == log-call order: the async queue is FIFO and drained by
            // a single worker, so a counter incremented here (the sole JSON
            // formatter, on the file sink only) reflects the order lines were
            // logged — a stable tiebreaker when two records share a millisecond.
            static std::atomic<std::uint64_t> seq_counter_;

            static void append_escaped(spdlog::memory_buf_t& dest, std::string_view s) {
                auto out = std::back_inserter(dest);
                for (char ch : s) {
                    const unsigned char c = static_cast<unsigned char>(ch);
                    switch (c) {
                        case '"':  put(dest, "\\\""); break;
                        case '\\': put(dest, "\\\\"); break;
                        case '\n': put(dest, "\\n"); break;
                        case '\r': put(dest, "\\r"); break;
                        case '\t': put(dest, "\\t"); break;
                        case '\b': put(dest, "\\b"); break;
                        case '\f': put(dest, "\\f"); break;
                        default:
                            if (c < 0x20) {
                                fmt::format_to(out, "\\u{:04x}", static_cast<int>(c));
                            } else {
                                dest.push_back(ch);
                            }
                    }
                }
            }

            static void put(spdlog::memory_buf_t& dest, const char* lit) {
                dest.append(lit, lit + std::strlen(lit));
            }
        };

        std::atomic<std::uint64_t> JsonLineFormatter::seq_counter_{0};

        // Compact UTC run id (e.g. "20260704T133226Z") — matches the toolchain's
        // .raccoon/runs/<run_id>/ scheme. A standalone run (no LIBSTP_LOG_DIR)
        // self-allocates a run directory with this id so its log is discovered the
        // same way as an orchestrated run's. There is no .raccoon/logs/ any more.
        std::string run_id_utc() {
            const auto now = std::chrono::system_clock::now();
            const std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm_buf{};
#if defined(_WIN32)
            gmtime_s(&tm_buf, &t);
#else
            gmtime_r(&t, &tm_buf);
#endif
            char stamp[32];
            std::strftime(stamp, sizeof(stamp), "%Y%m%dT%H%M%SZ", &tm_buf);
            return stamp;
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

        // Every run's log lives at .raccoon/runs/<run_id>/libstp.jsonl, co-located
        // with the run's localization/profile artifacts as one downloadable bundle.
        // `raccoon run` allocates the run dir and passes it via LIBSTP_LOG_DIR; a
        // standalone run (e.g. `uv run start`) self-allocates one with a UTC run id.
        std::filesystem::path log_dir;
        if (const char* run_log_dir = std::getenv("LIBSTP_LOG_DIR");
            run_log_dir && *run_log_dir) {
            log_dir = run_log_dir;
        } else {
            log_dir = std::filesystem::path(".raccoon/runs") / run_id_utc();
        }
        std::filesystem::create_directories(log_dir);
        const std::string log_path = (log_dir / "libstp.jsonl").string();

        // Console sink: warn + error only. The operator watches stdout for problems
        // during a live run; everything else is for post-hoc analysis and lives in
        // the JSONL file. spdlog's per-sink level filter does this natively — no
        // custom filter sink needed.
        auto console_color = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_color->set_level(spdlog::level::warn);
        auto console_fmt = std::make_unique<spdlog::pattern_formatter>();
        console_fmt->set_pattern("%H:%M:%S %^%l%$ %v");
        console_color->set_formatter(std::move(console_fmt));

        // File sink: JSONL, capturing everything the compile-time gate lets through
        // (DEBUG+, or TRACE+ with LIBSTP_TRACE_LOGGING). One JSON object per line,
        // all metadata dumped raw. Target computed above (the run dir's libstp.jsonl).
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path);
        file_sink->set_level(spdlog::level::trace);
        file_sink->set_formatter(spdlog::details::make_unique<JsonLineFormatter>());

        // Single core ASYNC logger, used as default for the whole app.
        //
        // The actual sink writes — stdout (a pipe / journald on the Pi) and the
        // per-run file on the SD card — must NOT run on the caller. Steps log
        // their signature via self.info() *on the asyncio mission loop*; a
        // synchronous write that blocks on an SD write-back would stall the entire
        // mission loop (the same hot-path stall that hit the stm32-data-reader). A
        // background worker drains a bounded queue; overrun_oldest drops the oldest
        // line when the queue is full instead of blocking the caller — losing a log
        // line beats stalling the robot.
        spdlog::init_thread_pool(8192, 1);
        auto logger = std::make_shared<spdlog::async_logger>(
            "core", spdlog::sinks_init_list{console_color, file_sink},
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

    bool is_enabled(Level /*level*/) {
        // No runtime level filtering: what is compiled in (the FOUNDATION_LOG_
        // ACTIVE_LEVEL gate at the call site) is captured. This only guards the
        // live state so importing the library alone spawns nothing, and log calls
        // during/after shutdown are dropped (avoids re-init of spdlog during C++
        // static teardown when atexit-driven motor disarms fire).
        return logger_initialized && !logger_shutdown.load(std::memory_order_relaxed);
    }

    void log(Level level, std::string_view message) {
        if (!is_enabled(level)) {
            return;
        }
        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return;
        }
        logger->log(to_spdlog_level(level), message);
    }

    void log(Level level, const char* source_file, int line, const char* func,
             std::string_view message) {
        if (!is_enabled(level)) {
            return;
        }
        auto logger = spdlog::default_logger_raw();
        if (!logger) {
            return;
        }
        // Carry source location in the log message (source_loc) rather than a
        // thread-local: the async backend formats on a worker thread. The pointers
        // must outlive the queued record, so intern them into a process-lifetime
        // pool — a caller-provided pointer (e.g. a transient Python str) may
        // otherwise be freed before the worker formats it.
        logger->log(spdlog::source_loc{intern_string(source_file), line, intern_string(func)},
                    to_spdlog_level(level), message);
    }

    void shutdown() {
        logger_shutdown.store(true, std::memory_order_relaxed);
        spdlog::shutdown();
        logger_initialized = false;
    }

} // namespace logging
