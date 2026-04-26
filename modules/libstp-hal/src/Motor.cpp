//
// Created by tobias on 6/9/25.
//
#include "hal/Motor.hpp"
#include "foundation/config.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

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
    LIBSTP_LOG_DEBUG("Destroying motor on port {}", port_);
    brake();
#ifdef SAFETY_CHECKS_ENABLED
    unregisterMotorPort(port_);
#endif
}

namespace {
std::atomic_bool    g_disabled{false};
// Accessed from both the signal handler (async-signal-safe write) and the
// watchdog thread (read). volatile sig_atomic_t is the only type the standard
// guarantees is safely readable/writable from a signal handler.
volatile sig_atomic_t g_signal_pending{0};

void disable_all_once() noexcept
{
    // Ensure we only attempt shutdown once per module copy.
    bool expected = false;
    if (!g_disabled.compare_exchange_strong(expected, true))
        return;
    libstp::hal::motor::Motor::disableAll();
}

void signal_handler(int signum)
{
    // Motor::disableAll() is not async-signal-safe (it may acquire locks or
    // call non-reentrant platform APIs). Only set a flag here; the watchdog
    // thread observes it and performs the actual cleanup.
    g_signal_pending = signum;
}

struct MotorFailSafe
{
    std::atomic_bool  watchdog_running{true};
    std::thread       watchdog;

    MotorFailSafe()
    {
        std::atexit(disable_all_once);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        watchdog = std::thread([this]() {
            using namespace std::chrono_literals;
            while (watchdog_running.load(std::memory_order_relaxed))
            {
                const int sig = g_signal_pending;
                if (sig != 0)
                {
                    disable_all_once();
                    std::signal(sig, SIG_DFL);
                    std::raise(sig);
                    break;
                }
                std::this_thread::sleep_for(5ms);
            }
        });
    }

    ~MotorFailSafe()
    {
        watchdog_running = false;
        if (watchdog.joinable())
            watchdog.join();
        disable_all_once();
    }
};

// Instantiate once per linked module to ensure cleanup on unload.
static MotorFailSafe g_motor_failsafe{};
} // namespace
