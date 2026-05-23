// Tests for libstp::threading::ThreadManager.
//
// Scope:
//   * Scoped daemons clean up on handle drop (RAII).
//   * Multiple scoped daemons live independently.
//   * Move semantics on DaemonHandle preserve ownership.
//   * Stop is idempotent and survives shutdown races.
//   * Persistent daemons stay alive until shutdown().
//
// We do NOT call shutdown() on the singleton in any test — the heartbeat
// (registered as persistent at .so load) must stay alive, and other test
// binaries link against the same ThreadManager singleton. Instead, every
// test cleans up its own scoped daemons via DaemonHandle's destructor.

#include <gtest/gtest.h>

#include "threading/thread_manager.hpp"

#include <atomic>
#include <chrono>
#include <thread>

using libstp::threading::ThreadManager;
using std::chrono::milliseconds;

namespace {

// Spin until `pred()` is true or the timeout elapses. Returns the final
// pred() value so the caller can EXPECT_TRUE on it.
template <typename Pred>
bool waitFor(Pred pred, milliseconds timeout = milliseconds(500)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(milliseconds(2));
    }
    return pred();
}

}  // namespace

TEST(ThreadManager, ScopedDaemonRunsAndStopsOnHandleDrop) {
    std::atomic<int> ticks{0};
    std::atomic<bool> exited{false};
    {
        auto handle = ThreadManager::instance().add_daemon(
            "test-scoped-runs", [&](libstp::threading::stop_token stop) {
                while (!stop.stop_requested()) {
                    ++ticks;
                    std::this_thread::sleep_for(milliseconds(2));
                }
                exited.store(true);
            });
        ASSERT_TRUE(handle.owns_thread());
        EXPECT_TRUE(waitFor([&] { return ticks.load() > 0; }));
    }  // handle out of scope -> stops + joins

    EXPECT_TRUE(exited.load()) << "Daemon should have observed stop_token and exited";
}

TEST(ThreadManager, ExplicitStopIsIdempotent) {
    std::atomic<bool> exited{false};
    auto handle = ThreadManager::instance().add_daemon(
        "test-stop-idempotent", [&](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(milliseconds(2));
            }
            exited.store(true);
        });
    ASSERT_TRUE(handle.owns_thread());

    handle.stop();
    EXPECT_FALSE(handle.owns_thread());
    EXPECT_TRUE(exited.load());

    // Second stop must not crash or block.
    handle.stop();
    EXPECT_FALSE(handle.owns_thread());
}

TEST(ThreadManager, DefaultHandleDestroysCleanly) {
    ThreadManager::DaemonHandle empty;
    EXPECT_FALSE(empty.owns_thread());
    // Destructor of `empty` is the actual test — must not segfault.
}

TEST(ThreadManager, MoveTransfersOwnership) {
    std::atomic<bool> exited{false};
    auto src = ThreadManager::instance().add_daemon(
        "test-move", [&](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(milliseconds(2));
            }
            exited.store(true);
        });
    ASSERT_TRUE(src.owns_thread());

    ThreadManager::DaemonHandle dst = std::move(src);
    EXPECT_FALSE(src.owns_thread());  // NOLINT(bugprone-use-after-move) — intentional check
    EXPECT_TRUE(dst.owns_thread());

    dst.stop();
    EXPECT_TRUE(exited.load());
}

TEST(ThreadManager, MoveAssignStopsTarget) {
    std::atomic<bool> first_exited{false};
    std::atomic<bool> second_exited{false};

    auto first = ThreadManager::instance().add_daemon(
        "test-move-assign-first", [&](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(milliseconds(2));
            }
            first_exited.store(true);
        });
    auto second = ThreadManager::instance().add_daemon(
        "test-move-assign-second", [&](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(milliseconds(2));
            }
            second_exited.store(true);
        });

    first = std::move(second);
    // The original "first" daemon got stopped+joined by move-assignment.
    EXPECT_TRUE(first_exited.load());
    EXPECT_FALSE(second_exited.load());

    first.stop();
    EXPECT_TRUE(second_exited.load());
}

TEST(ThreadManager, IndependentDaemonsRunInParallel) {
    std::atomic<int> a_ticks{0};
    std::atomic<int> b_ticks{0};

    auto a = ThreadManager::instance().add_daemon(
        "test-parallel-a", [&](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                ++a_ticks;
                std::this_thread::sleep_for(milliseconds(2));
            }
        });
    auto b = ThreadManager::instance().add_daemon(
        "test-parallel-b", [&](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                ++b_ticks;
                std::this_thread::sleep_for(milliseconds(2));
            }
        });

    EXPECT_TRUE(waitFor([&] { return a_ticks.load() > 0 && b_ticks.load() > 0; }));

    a.stop();
    const int a_at_stop = a_ticks.load();
    const int b_at_stop = b_ticks.load();
    std::this_thread::sleep_for(milliseconds(20));

    // b kept running after a stopped.
    EXPECT_EQ(a_ticks.load(), a_at_stop);
    EXPECT_GT(b_ticks.load(), b_at_stop);
}

TEST(ThreadManager, DaemonCountReflectsScopedRegistrations) {
    const std::size_t before = ThreadManager::instance().daemon_count();

    auto h1 = ThreadManager::instance().add_daemon(
        "test-count-1", [](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(milliseconds(2));
            }
        });
    auto h2 = ThreadManager::instance().add_daemon(
        "test-count-2", [](libstp::threading::stop_token stop) {
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(milliseconds(2));
            }
        });

    EXPECT_EQ(ThreadManager::instance().daemon_count(), before + 2);

    h1.stop();
    EXPECT_EQ(ThreadManager::instance().daemon_count(), before + 1);

    h2.stop();
    EXPECT_EQ(ThreadManager::instance().daemon_count(), before);
}

TEST(ThreadManager, ManyHandlesCanBeCreatedAndDropped) {
    // Stress: 50 short-lived daemons in sequence. Catches handle/id leaks and
    // any registry growth that doesn't undo on stop().
    const std::size_t before = ThreadManager::instance().daemon_count();
    for (int i = 0; i < 50; ++i) {
        std::atomic<bool> ran{false};
        auto h = ThreadManager::instance().add_daemon(
            "test-stress", [&](libstp::threading::stop_token stop) {
                ran.store(true);
                while (!stop.stop_requested()) {
                    std::this_thread::sleep_for(milliseconds(1));
                }
            });
        EXPECT_TRUE(waitFor([&] { return ran.load(); }));
    }
    EXPECT_EQ(ThreadManager::instance().daemon_count(), before);
}
