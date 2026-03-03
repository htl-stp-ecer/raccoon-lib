//
// Created by tobias on 1/30/25.
//

#pragma once

#include <coroutine>

namespace libstp::async
{
    /// Minimal coroutine wrapper that stores the last yielded value and supports manual stepping.
    template <typename T>
    class AsyncAlgorithm
    {
    public:
        /// Coroutine promise storing the most recent yielded or returned value.
        struct promise_type
        {
            bool initialized = false;

            T current_value{};

            AsyncAlgorithm get_return_object();

            static std::suspend_always initial_suspend() noexcept;

            static std::suspend_always final_suspend() noexcept;

            void return_value(const T& value);

            static void unhandled_exception();

            std::suspend_always yield_value(const T& value);
        };

        using handle_type = std::coroutine_handle<promise_type>;

        explicit AsyncAlgorithm(handle_type h);

        AsyncAlgorithm(const AsyncAlgorithm&) = delete;

        AsyncAlgorithm(AsyncAlgorithm&& other) noexcept;

        ~AsyncAlgorithm();

        /// Resume the coroutine once. Returns false when the coroutine is finished.
        bool advance();

        /// Return the last yielded or returned value from the coroutine.
        T current() const;

        /// Awaiter API: reports ready only once the wrapped coroutine is done.
        [[nodiscard]] bool await_ready() const noexcept;

        /// Awaiter API: resume the wrapped coroutine, then resume the awaiting coroutine.
        void await_suspend(std::coroutine_handle<> awaiting_coro) noexcept;

        /// Awaiter API: return the current value after suspension completes.
        T await_resume() noexcept;

    private:
        handle_type coro;
    };
}
