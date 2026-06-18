#include <chrono>
#include <cstdint>

#include "raccoon/Channels.h"
#include "raccoon/Transport.h"
#include "raccoon/scalar_i32_t.hpp"

namespace
{
    // Transport-units watchdog feed. Lazily created on first publish so merely
    // loading the bundle never opens a transport.
    raccoon::Transport& heartbeat_transport()
    {
        static raccoon::Transport transport = raccoon::Transport::create();
        return transport;
    }
}

// Wombat bundle: publish a scalar carrying our PID to the STM32 heartbeat
// channel. Called from the bundle-agnostic heartbeat daemon in raccoon.hal.
extern "C" bool raccoon_platform_heartbeat_publish(std::int32_t pid)
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    msg.value = pid;
    return heartbeat_transport().publish(raccoon::Channels::HEARTBEAT_CMD, msg);
}
