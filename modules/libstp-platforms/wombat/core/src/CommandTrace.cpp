#include "core/CommandTrace.hpp"

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <string>

using namespace platform::wombat::core;

namespace
{
    int64_t steadyNowNsec()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // Interpret an env var as a boolean. Accepts 1/true/yes/on (any case);
    // everything else (including unset, empty, and 0/false/no/off) is false.
    bool envIsTruthy(const char* value)
    {
        if (value == nullptr || value[0] == '\0')
            return false;
        std::string v;
        for (const char* p = value; *p != '\0'; ++p)
            v.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(*p))));
        return v == "1" || v == "true" || v == "yes" || v == "on";
    }
}

CommandTrace& CommandTrace::instance()
{
    static CommandTrace tracer;
    return tracer;
}

CommandTrace::CommandTrace()
{
    // RACCOON_CMD_TRACE is now a boolean flag, not a path. When truthy, the
    // trace is written into the run's artifact directory (LIBSTP_LOG_DIR, the
    // same dir the C++ logger writes libstp.jsonl into) as cmd_trace.robot.jsonl
    // so `raccoon logs` downloads it alongside the rest of the run bundle. This
    // is the send-side counterpart to the reader's receive-side cmd_trace.jsonl.
    if (!envIsTruthy(std::getenv("RACCOON_CMD_TRACE")))
        return;

    const char* logDir = std::getenv("LIBSTP_LOG_DIR");
    std::string path = (logDir != nullptr && logDir[0] != '\0')
                           ? std::string(logDir) + "/cmd_trace.robot.jsonl"
                           : std::string("cmd_trace.robot.jsonl");
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
    // Flush periodically, NOT per line: this is called from the mission's
    // asyncio loop thread, and a per-line flush on the Pi SD card can block it
    // for hundreds of ms under write-back pressure. Every 64 records bounds
    // loss on a crash to <64 lines while keeping the publish path off the disk.
    if ((seq & 0x3F) == 0)
        out_.flush();
}
