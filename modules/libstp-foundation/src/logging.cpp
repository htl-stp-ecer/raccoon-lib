//
// Created by tobias on 11/27/25.
//
#include <chrono>
#include <filesystem>
#include <optional>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/pattern_formatter.h>

namespace logging {

    namespace {
        std::optional<std::chrono::steady_clock::time_point> start_time;
        bool logger_initialized = false;
    }

    /// Call this if you ever want to reset the relative timer at runtime.
    void initialize_timer() {
        start_time = std::chrono::steady_clock::now();
        spdlog::info("Logging timer initialized");
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
                fmt::format_to(std::back_inserter(dest), "{:.3f}s", elapsed);
            } else {
                fmt::format_to(std::back_inserter(dest), "0.000s");
            }
        }

        std::unique_ptr<custom_flag_formatter> clone() const override {
            return spdlog::details::make_unique<ElapsedTimeFormatter>();
        }
    };

    void init() {
        if (logger_initialized) {
            spdlog::warn("logging::init() called multiple times; ignoring subsequent calls");
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
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::trace);

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            (log_dir / "libstp.log").string(),
            5 * 1024 * 1024,  // 5MB max file size
            3                 // keep 3 rotated files
        );
        file_sink->set_level(spdlog::level::trace);

        // Pattern formatter with custom elapsed-time flag '%E'.
        auto pattern_formatter = std::make_unique<spdlog::pattern_formatter>();
        pattern_formatter->add_flag<ElapsedTimeFormatter>('E');
        pattern_formatter->set_pattern(
            "[%Y-%m-%d %H:%M:%S] [+%E] [t%t/%n] [%^%l%$]: %v"
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

} // namespace logging