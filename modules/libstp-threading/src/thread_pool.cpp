#include "threading/thread_pool.hpp"

#include "foundation/logging.hpp"

#include <stdexcept>

namespace libstp::threading
{
    ThreadPool::ThreadPool(std::size_t worker_count)
    {
        if (worker_count == 0)
        {
            const auto hw = std::thread::hardware_concurrency();
            worker_count  = hw == 0 ? 2 : hw;
        }
        workers_.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i)
        {
            workers_.emplace_back(
                [this](std::stop_token stop) { worker_loop(stop); });
        }
        LIBSTP_LOG_DEBUG("ThreadPool: started with {} workers", worker_count);
    }

    ThreadPool::~ThreadPool()
    {
        shutdown();
    }

    void ThreadPool::enqueue(std::function<void()> job)
    {
        {
            std::lock_guard lk(mu_);
            if (stopping_)
                throw std::runtime_error("ThreadPool: submit after shutdown");
            queue_.push(std::move(job));
        }
        work_cv_.notify_one();
    }

    void ThreadPool::worker_loop(std::stop_token stop)
    {
        while (true)
        {
            std::function<void()> job;
            {
                std::unique_lock lk(mu_);
                work_cv_.wait(lk, stop, [this] { return !queue_.empty() || stopping_; });
                if (stop.stop_requested() && queue_.empty()) return;
                if (queue_.empty()) continue;
                job = std::move(queue_.front());
                queue_.pop();
                ++in_flight_;
            }

            try { job(); }
            catch (const std::exception& e)
            {
                LIBSTP_LOG_ERROR("ThreadPool: task threw: {}", e.what());
            }
            catch (...)
            {
                LIBSTP_LOG_ERROR("ThreadPool: task threw unknown exception");
            }

            {
                std::lock_guard lk(mu_);
                --in_flight_;
                if (queue_.empty() && in_flight_ == 0)
                    idle_cv_.notify_all();
            }
        }
    }

    void ThreadPool::wait_idle()
    {
        std::unique_lock lk(mu_);
        idle_cv_.wait(lk, [this] { return queue_.empty() && in_flight_ == 0; });
    }

    void ThreadPool::shutdown()
    {
        {
            std::lock_guard lk(mu_);
            if (stopping_) return;
            stopping_ = true;
        }
        for (auto& w : workers_) w.request_stop();
        work_cv_.notify_all();
        for (auto& w : workers_)
            if (w.joinable()) w.join();
        workers_.clear();
    }
}
