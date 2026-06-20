#include "core/CommandTrace.hpp"

#include <chrono>
#include <cstdlib>

using namespace platform::wombat::core;

namespace
{
    int64_t steadyNowNsec()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
}

CommandTrace& CommandTrace::instance()
{
    static CommandTrace tracer;
    return tracer;
}

CommandTrace::CommandTrace()
{
    const char* path = std::getenv("RACCOON_CMD_TRACE");
    if (path == nullptr || path[0] == '\0')
        return;
    out_.open(path, std::ios::out | std::ios::trunc);
    enabled_ = out_.is_open();
}

CommandTrace::~CommandTrace()
{
    if (out_.is_open())
        out_.flush();
}

void CommandTrace::record(const char* kind, const std::string& channel, int port, double v0,
                          double v1, double v2, int nvals, int64_t msgTimestampUsec)
{
    if (!enabled_)
        return;

    const uint64_t seq = seq_.fetch_add(1, std::memory_order_relaxed);
    const int64_t tNs = steadyNowNsec();

    std::lock_guard<std::mutex> lock(mutex_);
    out_ << "{\"t_ns\":" << tNs << ",\"seq\":" << seq << ",\"ts_us\":" << msgTimestampUsec
         << ",\"ch\":\"" << channel << "\",\"kind\":\"" << kind << "\",\"port\":" << port
         << ",\"v\":[";
    if (nvals > 0)
        out_ << v0;
    if (nvals > 1)
        out_ << ',' << v1;
    if (nvals > 2)
        out_ << ',' << v2;
    out_ << "]}\n";
    // Flush per line: an out-of-order bug often precedes a crash/hang, and the
    // trace is only enabled deliberately, so durability beats throughput here.
    out_.flush();
}
