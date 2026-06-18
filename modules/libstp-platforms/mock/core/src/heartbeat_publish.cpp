#include <cstdint>

// Mock bundle: there is no STM32 watchdog to feed, so the heartbeat publish is
// a no-op that still reports success. Defined here (rather than in the
// bundle-agnostic raccoon.hal extension) so the hal extension carries no
// raccoon-transport dependency. See modules/libstp-hal/bindings/heartbeat.cpp.
extern "C" bool raccoon_platform_heartbeat_publish(std::int32_t /*pid*/)
{
    return true;
}
