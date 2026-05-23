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

    ThreadManager::DaemonHandle
    ThreadManager::add_daemon(std::string name, DaemonFn fn)
    {
        std::lock_guard lk(mu_);
        if (shutdown_)
        {
            LIBSTP_LOG_WARN("ThreadManager: refusing to start daemon '{}' after shutdown", name);
            return {};
        }

        const std::uint64_t id = next_id_++;
        LIBSTP_LOG_DEBUG("ThreadManager: starting scoped daemon '{}' (id={})", name, id);
        daemons_.emplace_back(Daemon{
            id,
            std::move(name),
            jthread(std::move(fn)),
        });
        return DaemonHandle{this, id};
    }

    void ThreadManager::add_persistent_daemon(std::string name, DaemonFn fn)
    {
        std::lock_guard lk(mu_);
        if (shutdown_)
        {
            LIBSTP_LOG_WARN(
                "ThreadManager: refusing to start persistent daemon '{}' after shutdown", name);
            return;
        }
        LIBSTP_LOG_DEBUG("ThreadManager: starting persistent daemon '{}'", name);
        daemons_.emplace_back(Daemon{
            0,  // id=0 marks "not individually stoppable".
            std::move(name),
            jthread(std::move(fn)),
        });
    }

    void ThreadManager::stop_by_id(std::uint64_t id) noexcept
    {
        // Pop the matching daemon out from under the lock, then release the
        // lock before joining. Joining under the lock would deadlock if the
        // worker ever tries to call back into the ThreadManager (e.g. to
        // register a child daemon during teardown).
        Daemon victim{};
        bool   found = false;
        {
            std::lock_guard lk(mu_);
            for (auto it = daemons_.begin(); it != daemons_.end(); ++it)
            {
                if (it->id == id)
                {
                    victim = std::move(*it);
                    daemons_.erase(it);
                    found = true;
                    break;
                }
            }
        }

        if (!found)
        {
            // Either the daemon was already joined by a prior shutdown() or
            // the handle outlived its registration race-free. Either is fine.
            return;
        }

        LIBSTP_LOG_DEBUG("ThreadManager: stopping scoped daemon '{}' (id={})", victim.name, id);
        victim.thread.request_stop();
        if (victim.thread.joinable())
        {
            victim.thread.join();
        }
    }

    void ThreadManager::shutdown()
    {
        std::vector<Daemon> to_join;
        {
            std::lock_guard lk(mu_);
            if (shutdown_) return;
            shutdown_ = true;
            to_join   = std::move(daemons_);
        }

        for (auto& d : to_join)
        {
            d.thread.request_stop();
        }
        for (auto& d : to_join)
        {
            if (d.thread.joinable())
            {
                LIBSTP_LOG_DEBUG("ThreadManager: joining daemon '{}' (id={})", d.name, d.id);
                d.thread.join();
            }
        }
    }

    std::size_t ThreadManager::daemon_count() const
    {
        std::lock_guard lk(mu_);
        return daemons_.size();
    }

    ThreadManager::~ThreadManager()
    {
        shutdown();
    }
}
