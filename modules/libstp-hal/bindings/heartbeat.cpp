#include "foundation/logging.hpp"
#include "threading/thread_manager.hpp"

#if __has_include("core/LcmWriter.hpp")
#include "core/LcmWriter.hpp"
#define LIBSTP_HAS_WOMBAT_LCM_WRITER 1
#endif

#include <chrono>
#include <thread>

namespace
{
    // STM32 hardware watchdog expects a heartbeat every ~100 ms.
    constexpr auto kHeartbeatInterval = std::chrono::milliseconds(100);

    void heartbeat_loop(std::stop_token stop)
    {
        LIBSTP_LOG_DEBUG("Heartbeat daemon started");
        while (!stop.stop_requested())
        {
#if LIBSTP_HAS_WOMBAT_LCM_WRITER
            try
            {
                platform::wombat::core::LcmDataWriter::instance().sendHeartbeat();
            }
            catch (const std::exception& e)
            {
                LIBSTP_LOG_ERROR("Heartbeat publish failed: {}", e.what());
            }
#endif
            std::this_thread::sleep_for(kHeartbeatInterval);
        }
        LIBSTP_LOG_DEBUG("Heartbeat daemon stopping");
    }

    struct HeartbeatStarter
    {
        HeartbeatStarter()
        {
            libstp::threading::ThreadManager::instance().add_daemon(
                "heartbeat", &heartbeat_loop);
        }
    };

    // Spawned when the libstp_hal pybind11 extension is loaded; joined by
    // ThreadManager's destructor at process exit.
    const HeartbeatStarter g_heartbeat_starter{};
}
