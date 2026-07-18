#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace platform::wombat::core
{
    /**
     * Opt-in send-side command tracer.
     *
     * When the environment variable ``RACCOON_CMD_TRACE`` is truthy
     * (``1``/``true``/``yes``/``on``), every motor/servo/chassis command
     * published through :class:`TransportWriter` is appended as one JSON object
     * per line (JSONL) to ``cmd_trace.robot.jsonl`` inside the run's artifact
     * directory (``LIBSTP_LOG_DIR``), so ``raccoon logs`` downloads it with the
     * rest of the bundle. Otherwise the tracer is inert — :func:`enabled` is a
     * single bool read and ``record`` is never called by the writer.
     *
     * The point is *ordering*: a process-global monotonic ``seq`` is assigned to
     * every command across *all* channels, so the true intended send order (e.g.
     * "servo 1 before servo 0") is recoverable even though each port is its own
     * transport channel with no cross-channel ordering guarantee. The companion
     * receive-side tracer in stm32-data-reader logs the same commands as they
     * arrive and as they are pushed to SPI; the Python analyzer correlates the
     * two by ``(channel, ts_us)`` to flag commands that were *applied* out of the
     * order they were *sent*.
     *
     * Fields written per line:
     *   - ``t_ns``  steady-clock nanoseconds at publish (send-side latency base)
     *   - ``seq``   process-global monotonic command counter (intended order)
     *   - ``ts_us`` the message ``timestamp`` (system_clock µs) that also travels
     *               on the wire — the correlation key with the receive side
     *   - ``ch``    channel name
     *   - ``kind``  short command kind (``servo_pos``, ``motor_vel``, ...)
     *   - ``port``  device port, or -1 when not port-scoped (chassis/global)
     *   - ``v``     1-3 command values
     */
    class CommandTrace
    {
    public:
        static CommandTrace& instance();

        bool enabled() const { return enabled_; }

        /// Append one command record. ``nvals`` (1-3) selects how many of
        /// ``v0/v1/v2`` are meaningful. Cheap no-op when tracing is disabled.
        void record(const char* kind, const std::string& channel, int port, double v0,
                    double v1, double v2, int nvals, int64_t msgTimestampUsec);

    private:
        CommandTrace();
        ~CommandTrace();

        bool enabled_ = false;
        std::ofstream out_;
        std::mutex mutex_;
        std::atomic<uint64_t> seq_{0};
    };
}
