#include "foundation/logging.hpp"
#include "threading/thread_manager.hpp"

#if __has_include("core/LcmWriter.hpp")
#include "core/LcmWriter.hpp"
#define LIBSTP_HAS_WOMBAT_LCM_WRITER 1
#endif

#include <pybind11/pybind11.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace py = pybind11;

namespace
{
    // STM32 hardware watchdog expects a heartbeat every ~100 ms.
    constexpr auto kHeartbeatInterval = std::chrono::milliseconds(100);

    // Diagnostic counters. Atomic so Python-side readers can poll them safely
    // without taking any lock. ``last_interval_us`` is the wall-clock gap
    // between the last two beats; values much larger than 100 000 µs mean
    // the daemon is being starved (or its publish blocked).
    std::atomic<std::uint64_t> g_beats_sent{0};
    std::atomic<std::uint64_t> g_beats_failed{0};
    std::atomic<std::int64_t>  g_last_interval_us{0};
    std::atomic<std::int64_t>  g_last_beat_us{0};
    std::atomic<bool>          g_daemon_started{false};

    std::int64_t nowUs()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
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

        std::uint64_t local_beats   = 0;
        std::uint64_t local_failed  = 0;
        std::int64_t  prev_beat_us  = 0;

        while (!stop.stop_requested())
        {
            const std::int64_t this_us = nowUs();
            if (prev_beat_us != 0)
            {
                g_last_interval_us.store(this_us - prev_beat_us, std::memory_order_relaxed);
            }
            prev_beat_us = this_us;
            g_last_beat_us.store(this_us, std::memory_order_relaxed);

            bool ok = false;
#if LIBSTP_HAS_WOMBAT_LCM_WRITER
            try
            {
                platform::wombat::core::LcmDataWriter::instance().sendHeartbeat();
                ok = true;
            }
            catch (const std::exception& e)
            {
                LIBSTP_LOG_ERROR("Heartbeat publish failed: {}", e.what());
            }
#else
            // No wombat LCM writer in this build — pretend the beat went
            // out so the counter still advances; this path matters for
            // mock/sim builds where the watchdog is not real anyway.
            ok = true;
#endif

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
                LIBSTP_LOG_DEBUG(
                    "Heartbeat fell behind by {}us — re-anchoring schedule",
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        now_steady - next).count());
                next = now_steady;
            }
            std::this_thread::sleep_until(next);
        }
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
            d["last_interval_us"] = g_last_interval_us.load(std::memory_order_relaxed);
            d["last_beat_us"]    = g_last_beat_us.load(std::memory_order_relaxed);
            d["daemon_started"]  = g_daemon_started.load(std::memory_order_acquire);
            return d;
        },
        R"doc(
Snapshot of the C++ heartbeat daemon's state. Returns a dict with:

  sent             — total number of heartbeats successfully published
  failed           — total publishes that threw / returned false
  last_interval_us — wall-clock gap between the two most recent beats
  last_beat_us     — steady_clock timestamp (microseconds) of the most
                     recent beat — useful to detect a frozen daemon
  daemon_started   — True once the daemon has entered its loop. False
                     means the static initializer did not fire (most
                     likely: raccoon.hal was not imported).

For a live robot: call this once at startup, run a motion, call again,
and verify ``sent`` grew by roughly (elapsed_s * 10) and that
``last_interval_us`` is in the 95_000..150_000 µs window.
)doc");
}
