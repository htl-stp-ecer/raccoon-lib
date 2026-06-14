#include "foundation/logging.hpp"
#include "threading/thread_manager.hpp"

// Mock bundles have no raccoon-transport (raccoon::transport is an empty stub
// target), so the heartbeat cannot publish to the STM32 watchdog channel.
// Guard every transport touchpoint and keep the daemon/diagnostics scaffolding
// so `raccoon.hal` imports and the Python API stays identical across bundles.
#ifndef DRIVER_BUNDLE_MOCK
#include "raccoon/Channels.h"
#include "raccoon/Transport.h"
#include "raccoon/scalar_i32_t.hpp"
#endif

#include <pybind11/pybind11.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

namespace py = pybind11;

namespace
{
    // STM32 hardware watchdog expects a heartbeat every ~100 ms.
    constexpr auto kHeartbeatInterval = std::chrono::milliseconds(100);
    constexpr auto kOwnerRetryInterval = std::chrono::seconds(1);
    constexpr char kHeartbeatOwnerLockPath[] = "/tmp/raccoon_heartbeat_owner.lock";

    // Diagnostic counters. Atomic so Python-side readers can poll them safely
    // without taking any lock. ``last_interval_us`` is the wall-clock gap
    // between the last two beats; values much larger than 100 000 µs mean
    // the daemon is being starved (or its publish blocked).
    std::atomic<std::uint64_t> g_beats_sent{0};
    std::atomic<std::uint64_t> g_beats_failed{0};
    std::atomic<std::uint64_t> g_publish_slow_count{0};
    std::atomic<std::uint64_t> g_interval_miss_count{0};
    std::atomic<std::int64_t>  g_last_interval_us{0};
    std::atomic<std::int64_t>  g_last_beat_us{0};
    std::atomic<std::int64_t>  g_last_publish_us{0};
    std::atomic<std::int64_t>  g_max_publish_us{0};
    std::atomic<std::int64_t>  g_last_schedule_slip_us{0};
    std::atomic<std::int64_t>  g_max_schedule_slip_us{0};
    std::atomic<bool>          g_daemon_started{false};
    std::atomic<bool>          g_is_owner{false};
    std::atomic<int32_t>       g_owner_pid{0};

    constexpr std::int64_t kWarnIntervalUs = 150000;
    constexpr std::int64_t kWarnPublishUs = 20000;
    constexpr std::int64_t kWarnScheduleSlipUs = 20000;

    std::int64_t nowUs()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

#ifndef DRIVER_BUNDLE_MOCK
    raccoon::Transport& heartbeat_transport()
    {
        static raccoon::Transport transport = raccoon::Transport::create();
        return transport;
    }
#endif

    std::string processIdentity()
    {
        std::ostringstream out;
        out << "pid=" << static_cast<int>(::getpid());

        std::ifstream cmdline("/proc/self/cmdline", std::ios::binary);
        std::string payload;
        if (cmdline && std::getline(cmdline, payload, '\0'))
        {
            out << " cmd=" << payload;
        }

        return out.str();
    }

    std::string readOwnerIdentity()
    {
        std::ifstream in(kHeartbeatOwnerLockPath);
        std::string line;
        std::getline(in, line);
        return line;
    }

    class HeartbeatOwnerLock
    {
    public:
        bool ensureOwner()
        {
            if (fd_ >= 0)
            {
                return true;
            }

            const int fd = ::open(kHeartbeatOwnerLockPath, O_CREAT | O_RDWR, 0666);
            if (fd < 0)
            {
                LIBSTP_LOG_ERROR("Heartbeat owner lock open failed: {}", std::strerror(errno));
                return false;
            }

            if (::flock(fd, LOCK_EX | LOCK_NB) != 0)
            {
                ::close(fd);
                return false;
            }

            fd_ = fd;
            ownerIdentity_ = processIdentity();
            const std::string contents = ownerIdentity_ + "\n";
            ::ftruncate(fd_, 0);
            ::lseek(fd_, 0, SEEK_SET);
            (void)::write(fd_, contents.data(), contents.size());
            LIBSTP_LOG_WARN("Heartbeat ownership acquired by {}", ownerIdentity_);
            return true;
        }

        void release()
        {
            if (fd_ >= 0)
            {
                LIBSTP_LOG_WARN("Heartbeat ownership released by {}", ownerIdentity_);
                ::flock(fd_, LOCK_UN);
                ::close(fd_);
                fd_ = -1;
            }
        }

        bool isOwner() const
        {
            return fd_ >= 0;
        }

        const std::string& ownerIdentity() const
        {
            return ownerIdentity_;
        }

        ~HeartbeatOwnerLock()
        {
            release();
        }

    private:
        int fd_{-1};
        std::string ownerIdentity_{};
    };

    bool publish_heartbeat()
    {
#ifndef DRIVER_BUNDLE_MOCK
        raccoon::scalar_i32_t msg{};
        msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.value = static_cast<std::int32_t>(::getpid());
        return heartbeat_transport().publish(raccoon::Channels::HEARTBEAT_CMD, msg);
#else
        // No transport in mock bundles — there is no STM32 to feed, so the beat
        // is a no-op that still counts as "sent" to keep diagnostics coherent.
        return true;
#endif
    }

    void heartbeat_loop(libstp::threading::stop_token stop)
    {
        LIBSTP_LOG_DEBUG("Heartbeat daemon started (cadence={}ms)",
                         kHeartbeatInterval.count());
        g_daemon_started.store(true, std::memory_order_release);

        // Schedule-aware sleep so a slow ``sendHeartbeat()`` cannot drift
        // the cadence. ``sleep_until`` re-anchors each beat at ``next``;
        // ``next`` is then bumped by exactly the interval. If we fall behind
        // (next < now), we re-anchor to now so we don't dump a backlog of
        // beats all at once.
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        const auto period = kHeartbeatInterval;
        auto nextOwnerAttempt = next;

        std::uint64_t local_beats   = 0;
        std::uint64_t local_failed  = 0;
        std::int64_t  prev_beat_us  = 0;
        HeartbeatOwnerLock ownerLock;
        std::string lastObservedOwner;

        while (!stop.stop_requested())
        {
            if (!ownerLock.isOwner() && clock::now() >= nextOwnerAttempt)
            {
                nextOwnerAttempt = clock::now() + kOwnerRetryInterval;
                if (!ownerLock.ensureOwner())
                {
                    const std::string observedOwner = readOwnerIdentity();
                    if (!observedOwner.empty() && observedOwner != lastObservedOwner)
                    {
                        LIBSTP_LOG_WARN("Heartbeat ownership denied to pid={} because {} is active",
                                        static_cast<int>(::getpid()),
                                        observedOwner);
                        lastObservedOwner = observedOwner;
                    }
                }
            }

            g_is_owner.store(ownerLock.isOwner(), std::memory_order_relaxed);
            g_owner_pid.store(ownerLock.isOwner() ? static_cast<int32_t>(::getpid()) : 0,
                              std::memory_order_relaxed);
            if (!ownerLock.isOwner())
            {
                std::this_thread::sleep_until(next += period);
                continue;
            }

            const std::int64_t this_us = nowUs();
            if (prev_beat_us != 0)
            {
                const auto interval_us = this_us - prev_beat_us;
                g_last_interval_us.store(interval_us, std::memory_order_relaxed);
                if (interval_us > kWarnIntervalUs)
                {
                    const auto misses = g_interval_miss_count.fetch_add(1, std::memory_order_relaxed) + 1;
                    LIBSTP_LOG_WARN(
                        "Heartbeat interval miss: {}us between beats (miss_count={})",
                        interval_us,
                        misses);
                }
            }
            prev_beat_us = this_us;
            g_last_beat_us.store(this_us, std::memory_order_relaxed);

            bool ok = false;
            try
            {
                const auto publish_started = std::chrono::steady_clock::now();
                ok = publish_heartbeat();
                const auto publish_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - publish_started).count();
                g_last_publish_us.store(publish_us, std::memory_order_relaxed);
                g_max_publish_us.store(
                    std::max(g_max_publish_us.load(std::memory_order_relaxed), publish_us),
                    std::memory_order_relaxed);
                if (publish_us > kWarnPublishUs)
                {
                    const auto slow = g_publish_slow_count.fetch_add(1, std::memory_order_relaxed) + 1;
                    LIBSTP_LOG_WARN(
                        "Heartbeat publish blocked for {}us (slow_count={})",
                        publish_us,
                        slow);
                }
                if (!ok)
                {
                    LIBSTP_LOG_ERROR("Heartbeat publish returned false");
                }
            }
            catch (const std::exception& e)
            {
                LIBSTP_LOG_ERROR("Heartbeat publish failed: {}", e.what());
            }

            if (ok)
            {
                ++local_beats;
                g_beats_sent.store(local_beats, std::memory_order_relaxed);
            }
            else
            {
                ++local_failed;
                g_beats_failed.store(local_failed, std::memory_order_relaxed);
            }

            // Periodic liveness ping (every ~5 s) on the DEBUG channel so it
            // doesn't drown the operator-facing log stream. Real failures
            // already log at ERROR above and schedule slips at WARN below.
            if (local_beats % 50 == 1 && local_beats > 1)
            {
                LIBSTP_LOG_DEBUG("Heartbeat: sent={} failed={} last_interval={}us",
                                 local_beats, local_failed,
                                 g_last_interval_us.load(std::memory_order_relaxed));
            }

            next += period;
            const auto now_steady = clock::now();
            if (next < now_steady)
            {
                const auto slip_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    now_steady - next).count();
                g_last_schedule_slip_us.store(slip_us, std::memory_order_relaxed);
                g_max_schedule_slip_us.store(
                    std::max(g_max_schedule_slip_us.load(std::memory_order_relaxed), slip_us),
                    std::memory_order_relaxed);
                if (slip_us > kWarnScheduleSlipUs)
                {
                    LIBSTP_LOG_WARN(
                        "Heartbeat schedule slip: {}us behind cadence (last_publish={}us)",
                        slip_us,
                        g_last_publish_us.load(std::memory_order_relaxed));
                }
                else
                {
                    LIBSTP_LOG_DEBUG(
                        "Heartbeat fell behind by {}us — re-anchoring schedule",
                        slip_us);
                }
                next = now_steady;
            }
            std::this_thread::sleep_until(next);
        }
        g_is_owner.store(false, std::memory_order_relaxed);
        g_owner_pid.store(0, std::memory_order_relaxed);
        LIBSTP_LOG_DEBUG("Heartbeat daemon stopping after {} beats ({} failed)",
                         local_beats, local_failed);
    }

    struct HeartbeatStarter
    {
        HeartbeatStarter()
        {
            // Persistent: heartbeat lives as long as the library is loaded
            // and is joined by ThreadManager's destructor at process exit.
            libstp::threading::ThreadManager::instance().add_persistent_daemon(
                "heartbeat", &heartbeat_loop);
        }
    };

    const HeartbeatStarter g_heartbeat_starter{};
}

// Exported so the bindings module can attach Python-visible diagnostics.
void init_heartbeat(py::module& m)
{
    py::class_<std::tuple<std::uint64_t, std::uint64_t, std::int64_t, std::int64_t, bool>>(
        m, "_HeartbeatDiagTuple");  // unused placeholder so pybind doesn't whine

    m.def(
        "heartbeat_diagnostics",
        []()
        {
            // Returned as a plain dict so callers don't need a wrapper class.
            py::dict d;
            d["sent"]            = g_beats_sent.load(std::memory_order_relaxed);
            d["failed"]          = g_beats_failed.load(std::memory_order_relaxed);
            d["publish_slow_count"] = g_publish_slow_count.load(std::memory_order_relaxed);
            d["interval_miss_count"] = g_interval_miss_count.load(std::memory_order_relaxed);
            d["last_interval_us"] = g_last_interval_us.load(std::memory_order_relaxed);
            d["last_beat_us"]    = g_last_beat_us.load(std::memory_order_relaxed);
            d["last_publish_us"] = g_last_publish_us.load(std::memory_order_relaxed);
            d["max_publish_us"] = g_max_publish_us.load(std::memory_order_relaxed);
            d["last_schedule_slip_us"] = g_last_schedule_slip_us.load(std::memory_order_relaxed);
            d["max_schedule_slip_us"] = g_max_schedule_slip_us.load(std::memory_order_relaxed);
            d["daemon_started"]  = g_daemon_started.load(std::memory_order_acquire);
            d["is_owner"] = g_is_owner.load(std::memory_order_relaxed);
            d["owner_pid"] = g_owner_pid.load(std::memory_order_relaxed);
            return d;
        },
        R"doc(
Snapshot of the C++ heartbeat daemon's state. Returns a dict with:

  sent             — total number of heartbeats successfully published
  failed           — total publishes that threw / returned false
  publish_slow_count — publishes slower than 20 ms
  interval_miss_count — beat intervals larger than 150 ms
  last_interval_us — wall-clock gap between the two most recent beats
  last_beat_us     — steady_clock timestamp (microseconds) of the most
                     recent beat — useful to detect a frozen daemon
  last_publish_us  — time spent inside sendHeartbeat() for the last beat
  max_publish_us   — max observed sendHeartbeat() time
  last_schedule_slip_us — how far behind cadence the loop was on the last slip
  max_schedule_slip_us  — max observed cadence slip
  is_owner         — True only in the elected sender process
  owner_pid        — PID of the sender process in this process, else 0
  daemon_started   — True once the daemon has entered its loop. False
                     means the static initializer did not fire (most
                     likely: raccoon.hal was not imported).

For a live robot: call this once at startup, run a motion, call again,
and verify ``sent`` grew by roughly (elapsed_s * 10) and that
``last_interval_us`` is in the 95_000..150_000 µs window.
)doc");
}
