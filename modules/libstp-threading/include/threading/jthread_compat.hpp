#pragma once

// std::jthread + std::stop_token availability is fragmented. GCC, MSVC, and
// upstream libc++ 20 ship them, but Apple's libc++ (Xcode 16.4, clang 17) does
// not declare the types at all — _LIBCPP_DISABLE_AVAILABILITY can't help
// because the header is missing the symbols entirely.
//
// This compat layer aliases std:: on platforms that have the feature and
// provides a minimal RAII-stopping fallback on Apple. The public ABI matches
// what ThreadManager uses: jthread construction with a stop_token-taking
// callable, request_stop(), joinable(), join(), and stop_token::stop_requested().

#include <thread>

#if defined(__cpp_lib_jthread) && __cpp_lib_jthread >= 201911L

#    include <stop_token>

namespace libstp::threading::compat
{
    using std::jthread;
    using std::stop_source;
    using std::stop_token;
}

#else

#    include <atomic>
#    include <memory>
#    include <type_traits>
#    include <utility>

namespace libstp::threading::compat
{
    /// Cooperative stop signal observed by a daemon. Shares a refcounted flag
    /// with its stop_source; thread-safe.
    class stop_token
    {
    public:
        stop_token() = default;
        explicit stop_token(std::shared_ptr<std::atomic<bool>> flag) noexcept
            : flag_(std::move(flag))
        {
        }

        [[nodiscard]] bool stop_requested() const noexcept
        {
            return flag_ && flag_->load(std::memory_order_acquire);
        }

        [[nodiscard]] bool stop_possible() const noexcept
        {
            return static_cast<bool>(flag_);
        }

    private:
        std::shared_ptr<std::atomic<bool>> flag_;
    };

    /// Owner side of the stop signal. Setting it is sticky and atomic.
    class stop_source
    {
    public:
        stop_source() : flag_(std::make_shared<std::atomic<bool>>(false)) {}

        [[nodiscard]] stop_token get_token() const noexcept
        {
            return stop_token(flag_);
        }

        bool request_stop() noexcept
        {
            if (!flag_) return false;
            bool expected = false;
            return flag_->compare_exchange_strong(
                expected, true, std::memory_order_release, std::memory_order_acquire);
        }

        [[nodiscard]] bool stop_requested() const noexcept
        {
            return flag_ && flag_->load(std::memory_order_acquire);
        }

    private:
        std::shared_ptr<std::atomic<bool>> flag_;
    };

    /// Drop-in replacement for std::jthread: owns a std::thread plus a
    /// stop_source, requests stop and joins on destruction.
    class jthread
    {
    public:
        jthread() noexcept = default;

        template <typename F, typename... Args>
        explicit jthread(F&& f, Args&&... args)
        {
            if constexpr (std::is_invocable_v<F&, stop_token, Args&...>)
            {
                thread_ = std::thread(
                    [fn = std::forward<F>(f), tok = source_.get_token()](
                        auto&&... a) mutable {
                        fn(tok, std::forward<decltype(a)>(a)...);
                    },
                    std::forward<Args>(args)...);
            }
            else
            {
                thread_ = std::thread(std::forward<F>(f), std::forward<Args>(args)...);
            }
        }

        ~jthread()
        {
            if (thread_.joinable())
            {
                source_.request_stop();
                thread_.join();
            }
        }

        jthread(const jthread&)            = delete;
        jthread& operator=(const jthread&) = delete;

        jthread(jthread&& other) noexcept
            : source_(std::move(other.source_)), thread_(std::move(other.thread_))
        {
        }

        jthread& operator=(jthread&& other) noexcept
        {
            if (this != &other)
            {
                if (thread_.joinable())
                {
                    source_.request_stop();
                    thread_.join();
                }
                source_ = std::move(other.source_);
                thread_ = std::move(other.thread_);
            }
            return *this;
        }

        [[nodiscard]] bool joinable() const noexcept { return thread_.joinable(); }
        void               join() { thread_.join(); }
        void               detach() { thread_.detach(); }

        [[nodiscard]] stop_source get_stop_source() const noexcept { return source_; }
        [[nodiscard]] stop_token  get_stop_token() const noexcept
        {
            return source_.get_token();
        }
        bool request_stop() noexcept { return source_.request_stop(); }

    private:
        stop_source source_{};
        std::thread thread_{};
    };
}

#endif

// Re-export the active implementation directly under libstp::threading so the
// rest of the codebase writes unqualified `jthread` / `stop_token` and stays
// agnostic to whether std:: or the fallback is in use.
namespace libstp::threading
{
    using jthread     = compat::jthread;
    using stop_source = compat::stop_source;
    using stop_token  = compat::stop_token;
}
