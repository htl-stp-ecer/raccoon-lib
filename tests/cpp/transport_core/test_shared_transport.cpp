#include "transport_core/shared_transport.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace
{
    void wait_for(std::chrono::milliseconds duration)
    {
        std::this_thread::sleep_for(duration);
    }
}

TEST(SharedTransportTest, RepeatedSubscribeUnsubscribeDoesNotMultiplyUnderlyingDeliveries)
{
    auto& transport = libstp::transport_core::SharedTransport::instance();
    transport.shutdown();
    auto ignored = transport.get_and_reset_stats();
    (void)ignored;

    const std::string channel = "unit/shared-transport/churn";

    for (int i = 0; i < 100; ++i)
    {
        auto subscription = transport.subscribe_raw(
            channel,
            [](const void*, int) {},
            false,
            false);
        ASSERT_NE(subscription, 0U);
        transport.unsubscribe(subscription);
    }

    std::atomic<int> deliveries{0};
    auto live_subscription = transport.subscribe_raw(
        channel,
        [&deliveries](const void*, int) { deliveries.fetch_add(1); },
        false,
        false);
    ASSERT_NE(live_subscription, 0U);

    const std::uint8_t payload[] = {0x12, 0x34, 0x56, 0x78};
    constexpr int publish_count = 8;
    for (int i = 0; i < publish_count; ++i)
    {
        ASSERT_TRUE(transport.publish_raw(channel, payload, static_cast<int>(sizeof(payload)), false, false, 100, 10));
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (deliveries.load() < publish_count && std::chrono::steady_clock::now() < deadline)
    {
        wait_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(deliveries.load(), publish_count);

    auto stats = transport.get_and_reset_stats();
    ASSERT_EQ(transport.active_subscription_count(), 1U);

    const auto channel_it = std::find_if(
        stats.channels.begin(),
        stats.channels.end(),
        [&channel](const raccoon::TransportStats::Channel& entry)
        {
            return entry.name == channel;
        });

    ASSERT_NE(channel_it, stats.channels.end());
    EXPECT_EQ(channel_it->deliveries, publish_count);
    EXPECT_EQ(channel_it->callback.count, publish_count);

    transport.unsubscribe(live_subscription);
    EXPECT_EQ(transport.active_subscription_count(), 0U);
}

// shutdown() must leave the singleton usable: the raccoon-transport backend tears
// down its Node deterministically, and the very next subscribe / publish
// has to re-create it transparently. Regression for the migration period
// where `Transport::shutdown()` (destroys impl_) blocked every subsequent
// caller — fixed in SharedTransport::ensure_started() via lazy re-init.
TEST(SharedTransportTest, ShutdownIsReversibleViaLazyReinit)
{
    auto& transport = libstp::transport_core::SharedTransport::instance();

    // Tear down, then immediately try to use it again.
    transport.shutdown();
    auto ignored = transport.get_and_reset_stats();
    (void)ignored;

    const std::string channel = "unit/shared-transport/reinit";
    std::atomic<int> deliveries{0};
    auto sub = transport.subscribe_raw(
        channel,
        [&deliveries](const void*, int) { deliveries.fetch_add(1); },
        false, false);
    ASSERT_NE(sub, 0U) << "subscribe_raw must re-init the underlying transport "
                         "after shutdown(); it returned 0";

    const std::uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    ASSERT_TRUE(transport.publish_raw(channel, payload, sizeof(payload),
                                      false, false, 100, 10));

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (deliveries.load() < 1 && std::chrono::steady_clock::now() < deadline)
    {
        wait_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(deliveries.load(), 1);

    transport.unsubscribe(sub);
    EXPECT_EQ(transport.active_subscription_count(), 0U);
}

// The raccoon-transport backend uses a dynamic payload type, so payloads
// well above the old 4 KB WireFrame ceiling must round-trip intact.
// 256 KB matches the camera-frame magnitude that motivated the
// migration to bb::Slice<uint8_t>.
TEST(SharedTransportTest, LargePayloadRoundtripsIntact)
{
    auto& transport = libstp::transport_core::SharedTransport::instance();
    transport.shutdown();

    const std::string channel = "unit/shared-transport/big";
    constexpr std::size_t kSize = 256 * 1024; // 256 KB
    std::vector<std::uint8_t> tx(kSize);
    for (std::size_t i = 0; i < kSize; ++i)
    {
        tx[i] = static_cast<std::uint8_t>((i * 31u + 7u) & 0xFF);
    }

    std::atomic<bool> got_ok{false};
    std::atomic<int> rx_len{0};
    auto sub = transport.subscribe_raw(
        channel,
        [&](const void* data, int data_len)
        {
            rx_len.store(data_len);
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            for (int i = 0; i < data_len; ++i)
            {
                if (bytes[i] != static_cast<std::uint8_t>((i * 31u + 7u) & 0xFF))
                {
                    return; // mismatch ⇒ leave got_ok = false
                }
            }
            got_ok.store(true);
        },
        false, false);
    ASSERT_NE(sub, 0U);

    ASSERT_TRUE(transport.publish_raw(channel, tx.data(),
                                      static_cast<int>(tx.size()),
                                      false, false, 100, 10));

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!got_ok.load() && std::chrono::steady_clock::now() < deadline)
    {
        wait_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(rx_len.load(), static_cast<int>(kSize))
        << "subscriber callback received wrong payload length";
    EXPECT_TRUE(got_ok.load())
        << "payload bytes did not round-trip intact through the raccoon-transport payload";

    transport.unsubscribe(sub);
}

// Regression: dispatching subscriber callbacks under `apiMutex` deadlocked
// against SharedTransport's own `mu_` (subscribe() acquires mu_ → spin
// daemon was inside drainAll holding apiMutex → callback wants mu_).
// Fixed by snapshotting samples and dispatching lock-free. This test puts
// a fresh subscribe() call right next to a publish burst; pre-fix it
// hung indefinitely.
TEST(SharedTransportTest, ConcurrentPublishAndSubscribeDoesNotDeadlock)
{
    auto& transport = libstp::transport_core::SharedTransport::instance();
    transport.shutdown();

    const std::string busy_channel = "unit/shared-transport/busy";
    const std::string late_channel = "unit/shared-transport/late";

    std::atomic<bool> stop_publisher{false};
    auto publisher = std::thread([&]()
    {
        const std::uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
        while (!stop_publisher.load())
        {
            transport.publish_raw(busy_channel, payload, sizeof(payload),
                                  false, false, 100, 10);
            wait_for(std::chrono::milliseconds(1));
        }
    });

    auto busy_sub = transport.subscribe_raw(
        busy_channel, [](const void*, int) {}, false, false);
    ASSERT_NE(busy_sub, 0U);

    // Now in the middle of the publish/drain churn, register a *fresh*
    // subscriber on a *different* channel. If apiMutex were held across
    // dispatch this call would never return.
    auto late_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    std::uint64_t late_sub = 0;
    while (late_sub == 0 && std::chrono::steady_clock::now() < late_deadline)
    {
        late_sub = transport.subscribe_raw(
            late_channel, [](const void*, int) {}, false, false);
    }
    EXPECT_NE(late_sub, 0U)
        << "subscribe_raw never returned during a publish burst — apiMutex is "
           "being held across dispatch (deadlock with SharedTransport.mu_)";

    stop_publisher.store(true);
    publisher.join();
    transport.unsubscribe(busy_sub);
    if (late_sub) transport.unsubscribe(late_sub);
}
