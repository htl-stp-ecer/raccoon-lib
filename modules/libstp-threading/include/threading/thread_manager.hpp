#pragma once

#include "threading/jthread_compat.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace libstp::threading
{
    using DaemonFn = std::function<void(stop_token)>;

    /// Process-wide registry for background threads.
    ///
    /// Every long-lived worker in raccoon-lib goes through this singleton —
    /// nothing constructs a bare ``std::thread`` of its own. There are two
    /// lifetimes:
    ///
    ///   * **Persistent daemons** (``add_persistent_daemon``) live until the
    ///     library is unloaded; they are joined by ``shutdown()`` (called from
    ///     the singleton destructor). Use this for global services like the
    ///     STM32 heartbeat.
    ///
    ///   * **Scoped daemons** (``add_daemon``) return an RAII ``DaemonHandle``.
    ///     Dropping the handle requests stop on the underlying ``std::jthread``
    ///     and joins it. Use this for per-instance workers (e.g. the
    ///     Localization tick loop), where the thread's lifetime is tied to a
    ///     particular owning object instead of the process.
    ///
    /// Both flavours observe the same ``std::stop_token`` contract — daemons
    /// must return when the token is requested.
    class ThreadManager
    {
    public:
        class DaemonHandle;

        static ThreadManager& instance();

        /// Register a scoped daemon. The caller owns the returned handle;
        /// dropping it stops and joins the thread. Calling this after
        /// ``shutdown()`` returns an empty handle and logs a warning.
        [[nodiscard]] DaemonHandle add_daemon(std::string name, DaemonFn fn);

        /// Register a process-lifetime daemon. Joined at ``shutdown()`` time.
        /// Use this only when the thread must live as long as the library
        /// itself does — otherwise prefer ``add_daemon``.
        void add_persistent_daemon(std::string name, DaemonFn fn);

        /// Request stop on every registered daemon (persistent and scoped) and
        /// join them. Idempotent; safe to call from the singleton destructor
        /// and from teardown helpers in tests.
        void shutdown();

        /// Number of currently-registered daemons. Test helper.
        [[nodiscard]] std::size_t daemon_count() const;

        ~ThreadManager();
        ThreadManager(const ThreadManager&) = delete;
        ThreadManager& operator=(const ThreadManager&) = delete;

    private:
        friend class DaemonHandle;

        ThreadManager() = default;

        /// Stop and join the daemon identified by ``id``. Called from
        /// ``DaemonHandle``'s destructor. Silently no-ops if the daemon was
        /// already removed (e.g. by a prior ``shutdown()``).
        void stop_by_id(std::uint64_t id) noexcept;

        struct Daemon
        {
            std::uint64_t id;     ///< 0 means "persistent, not individually stoppable".
            std::string   name;
            jthread       thread;
        };

        mutable std::mutex  mu_;
        std::vector<Daemon> daemons_;
        std::uint64_t       next_id_  = 1;
        bool                shutdown_ = false;
    };

    /// Move-only RAII token returned by ``ThreadManager::add_daemon``. On
    /// destruction, requests stop on the underlying ``jthread`` and joins it,
    /// then removes it from the registry. A default-constructed handle owns
    /// nothing and is safe to destroy.
    class ThreadManager::DaemonHandle
    {
    public:
        DaemonHandle() noexcept = default;
        ~DaemonHandle() { stop(); }

        DaemonHandle(const DaemonHandle&)            = delete;
        DaemonHandle& operator=(const DaemonHandle&) = delete;

        DaemonHandle(DaemonHandle&& other) noexcept
            : mgr_(other.mgr_), id_(other.id_)
        {
            other.mgr_ = nullptr;
            other.id_  = 0;
        }

        DaemonHandle& operator=(DaemonHandle&& other) noexcept
        {
            if (this != &other)
            {
                stop();
                mgr_       = other.mgr_;
                id_        = other.id_;
                other.mgr_ = nullptr;
                other.id_  = 0;
            }
            return *this;
        }

        /// Stop and join the daemon now (idempotent). Subsequent calls and the
        /// destructor become no-ops.
        void stop() noexcept
        {
            if (mgr_ != nullptr && id_ != 0)
            {
                mgr_->stop_by_id(id_);
            }
            mgr_ = nullptr;
            id_  = 0;
        }

        [[nodiscard]] bool owns_thread() const noexcept { return mgr_ != nullptr && id_ != 0; }

    private:
        friend class ThreadManager;
        DaemonHandle(ThreadManager* mgr, std::uint64_t id) noexcept
            : mgr_(mgr), id_(id)
        {
        }

        ThreadManager* mgr_ = nullptr;
        std::uint64_t  id_  = 0;
    };
}
