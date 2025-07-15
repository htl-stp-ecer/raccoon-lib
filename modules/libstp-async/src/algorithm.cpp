//
// Created by tobias on 1/30/25.
//

#include "async/algorithm.hpp"

#include <exception>

template <typename T>
libstp::async::AsyncAlgorithm<T> libstp::async::AsyncAlgorithm<T>::promise_type::get_return_object()
{
    return AsyncAlgorithm(std::coroutine_handle<promise_type>::from_promise(*this));
}

template <typename T>
std::suspend_always libstp::async::AsyncAlgorithm<T>::promise_type::initial_suspend() noexcept
{
    return {};
}

template <typename T>
std::suspend_always libstp::async::AsyncAlgorithm<T>::promise_type::final_suspend() noexcept
{
    return {};
}

template <typename T>
void libstp::async::AsyncAlgorithm<T>::promise_type::return_value(const T& value)
{
    current_value = value;
}

template <typename T>
void libstp::async::AsyncAlgorithm<T>::promise_type::unhandled_exception()
{
    std::terminate();
}

template <typename T>
std::suspend_always libstp::async::AsyncAlgorithm<T>::promise_type::yield_value(const T& value)
{
    current_value = value;
    return {};
}

template <typename T>
libstp::async::AsyncAlgorithm<T>::AsyncAlgorithm(handle_type h): coro(h)
{
}

template <typename T>
libstp::async::AsyncAlgorithm<T>::AsyncAlgorithm(AsyncAlgorithm&& other) noexcept: coro(other.coro)
{
    other.coro = nullptr;
}

template <typename T>
libstp::async::AsyncAlgorithm<T>::~AsyncAlgorithm()
{
    if (coro)
    {
        coro.destroy();
    }
}

template <typename T>
bool libstp::async::AsyncAlgorithm<T>::advance()
{
    if (!coro || coro.done())
        return false;

    auto& p = coro.promise();

    if (!p.initialized)
    {
        p.initialized = true;
    }

    coro.resume();

    if (coro.done())
        return false;

    return true;
}

template <typename T>
T libstp::async::AsyncAlgorithm<T>::current() const
{
    return coro.promise().current_value;
}

template <typename T>
bool libstp::async::AsyncAlgorithm<T>::await_ready() const noexcept
{
    // Ready if the coroutine has already produced a value.
    return coro.done();
}

template <typename T>
void libstp::async::AsyncAlgorithm<T>::await_suspend(const std::coroutine_handle<> awaiting_coro) noexcept
{
    // Resume the current coroutine and then resume the awaiting coroutine.
    if (coro && !coro.done())
    {
        coro.resume();
    }
    awaiting_coro.resume();
}

template <typename T>
T libstp::async::AsyncAlgorithm<T>::await_resume() noexcept
{
    return current();
}
