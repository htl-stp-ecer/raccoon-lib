#pragma once

#include <functional>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace libstp::threading
{
    using DaemonFn = std::function<void(std::stop_token)>;

    /// Process-wide registry for long-running background threads.
    ///
    /// A `daemon` is a thread that lives as long as the library is loaded.
    /// Each daemon receives a `std::stop_token` and must observe it to
    /// terminate cleanly. On `shutdown()` (called automatically when the
    /// singleton is destroyed) every daemon is sent a stop request and
    /// joined.
    ///
    /// Construct daemons from a static initializer (or from inside the
    /// pybind11 module init) so they start as soon as the .so is loaded.
    class ThreadManager
    {
    public:
        static ThreadManager& instance();

        /// Register and start a daemon. The callable is invoked on a new
        /// std::jthread and must return when its stop_token is requested.
        void add_daemon(std::string name, DaemonFn fn);

        /// Request stop on all daemons and join them. Idempotent.
        void shutdown();

        ~ThreadManager();

        ThreadManager(const ThreadManager&) = delete;
        ThreadManager& operator=(const ThreadManager&) = delete;

    private:
        ThreadManager() = default;

        struct Daemon
        {
            std::string  name;
            std::jthread thread;
        };

        std::mutex          mu_;
        std::vector<Daemon> daemons_;
        bool                shutdown_ = false;
    };
}
