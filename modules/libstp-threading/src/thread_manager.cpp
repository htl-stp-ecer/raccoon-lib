#include "threading/thread_manager.hpp"

#include "foundation/logging.hpp"

#include <utility>

namespace libstp::threading
{
    ThreadManager& ThreadManager::instance()
    {
        static ThreadManager mgr;
        return mgr;
    }

    void ThreadManager::add_daemon(std::string name, DaemonFn fn)
    {
        std::lock_guard lk(mu_);
        if (shutdown_)
        {
            LIBSTP_LOG_WARN("ThreadManager: refusing to start daemon '{}' after shutdown", name);
            return;
        }
        LIBSTP_LOG_DEBUG("ThreadManager: starting daemon '{}'", name);
        daemons_.emplace_back(Daemon{
            std::move(name),
            std::jthread(std::move(fn)),
        });
    }

    void ThreadManager::shutdown()
    {
        std::vector<Daemon> to_join;
        {
            std::lock_guard lk(mu_);
            if (shutdown_) return;
            shutdown_ = true;
            to_join = std::move(daemons_);
        }

        for (auto& d : to_join)
            d.thread.request_stop();

        for (auto& d : to_join)
        {
            if (d.thread.joinable())
            {
                LIBSTP_LOG_DEBUG("ThreadManager: joining daemon '{}'", d.name);
                d.thread.join();
            }
        }
    }

    ThreadManager::~ThreadManager()
    {
        shutdown();
    }
}
