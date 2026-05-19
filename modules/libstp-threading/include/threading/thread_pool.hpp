#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stop_token>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace libstp::threading
{
    /// Fixed-size worker pool for short-lived tasks.
    ///
    /// Workers are std::jthreads; the destructor requests stop, drains the
    /// queue, and joins. Submitted tasks return a std::future so callers can
    /// await results or exceptions.
    ///
    /// Intended for fan-out compute (e.g. parallel motion-controller updates
    /// or sensor preprocessing). It is NOT a general-purpose async runtime —
    /// blocking I/O or unbounded waits will starve the pool.
    class ThreadPool
    {
    public:
        explicit ThreadPool(std::size_t worker_count = 0);
        ~ThreadPool();

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        /// Submit a callable; returns a future for the result.
        /// Throws std::runtime_error if the pool has already been shut down.
        template <typename F, typename... Args>
        auto submit(F&& fn, Args&&... args)
            -> std::future<std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>>
        {
            using R = std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>;
            auto task = std::make_shared<std::packaged_task<R()>>(
                [fn = std::forward<F>(fn),
                 tup = std::make_tuple(std::forward<Args>(args)...)]() mutable
                {
                    return std::apply(std::move(fn), std::move(tup));
                });
            auto fut = task->get_future();
            enqueue([task]() { (*task)(); });
            return fut;
        }

        /// Block until the queue is empty and no tasks are in flight.
        void wait_idle();

        /// Stop accepting new tasks and join all workers. Idempotent.
        void shutdown();

        [[nodiscard]] std::size_t worker_count() const noexcept { return workers_.size(); }

    private:
        void enqueue(std::function<void()> job);
        void worker_loop(std::stop_token stop);

        mutable std::mutex                 mu_;
        std::condition_variable_any        work_cv_;
        std::condition_variable_any        idle_cv_;
        std::queue<std::function<void()>>  queue_;
        std::size_t                        in_flight_ = 0;
        bool                               stopping_  = false;
        std::vector<std::jthread>          workers_;
    };
}
