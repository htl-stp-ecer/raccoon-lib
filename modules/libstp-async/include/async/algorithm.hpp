//
// Created by tobias on 1/30/25.
//

#pragma once

#include <coroutine>

namespace libstp::async
{
    template <typename T>
    class AsyncAlgorithm
    {
    public:
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

        bool advance();

        T current() const;

        [[nodiscard]] bool await_ready() const noexcept;

        void await_suspend(std::coroutine_handle<> awaiting_coro) noexcept;

        T await_resume() noexcept;

    private:
        handle_type coro;
    };
}
