//
// Created by tobias on 6/9/25.
//
#include "hal/Motor.hpp"
#include "foundation/config.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>

using namespace libstp::hal::motor;

#ifdef SAFETY_CHECKS_ENABLED
void Motor::registerMotorPort(int port)
{
    LIBSTP_LOG_DEBUG("Registering motor port {}", port);
    if (used_motor_ports.contains(port))
    {
        LIBSTP_LOG_WARN("Motor port {} is already in use!", port);
        return;
    }

    used_motor_ports.insert(port);
}

void Motor::unregisterMotorPort(const int port)
{
    LIBSTP_LOG_DEBUG("Unregistering motor port {}", port);
    used_motor_ports.erase(port);
}
#endif


Motor::~Motor()
{
    LIBSTP_LOG_DEBUG("Destroying motor on port {}", port);
    brake();
#ifdef SAFETY_CHECKS_ENABLED
    unregisterMotorPort(port);
#endif
}

namespace {
std::atomic_bool g_disabled{false};

void disable_all_once() noexcept
{
    // Ensure we only attempt shutdown once per module copy.
    bool expected = false;
    if (!g_disabled.compare_exchange_strong(expected, true))
        return;
    try
    {
        libstp::hal::motor::Motor::disableAll();
    }
    catch (...)
    {
        // Swallow errors; best-effort shutdown.
    }
}

void signal_handler(int signum)
{
    disable_all_once();
    // Chain to default handler after making motors safe.
    std::signal(signum, SIG_DFL);
    std::raise(signum);
}

struct MotorFailSafe
{
    MotorFailSafe()
    {
        std::atexit(disable_all_once);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
    }
    ~MotorFailSafe()
    {
        disable_all_once();
    }
};

// Instantiate once per linked module to ensure cleanup on unload.
static MotorFailSafe g_motor_failsafe{};
} // namespace
