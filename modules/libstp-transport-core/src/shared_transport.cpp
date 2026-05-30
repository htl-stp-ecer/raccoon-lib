#include "transport_core/shared_transport.hpp"

#include "foundation/logging.hpp"
#include "raccoon/Channels.h"

#include <chrono>
#include <cstring>
#include <vector>

// SharedTransport forwards the legacy `reliable` flag (and its retry/timeout
// siblings) into raccoon::PublishOptions for backwards compatibility with
// existing call sites. The iceoryx2 backend silently ignores them. The
// deprecation attribute on those fields is for *external* callers; suppress
// it on our own pass-through reads.
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

namespace
{
    std::vector<std::uint8_t> encode_retain_request(const std::string& channel)
    {
        const auto channel_len = static_cast<std::int32_t>(channel.size());
        std::vector<std::uint8_t> payload(static_cast<std::size_t>(8 + 8 + 4 + channel_len + 4), 0);
        int offset = 16;
        payload[offset++] = static_cast<std::uint8_t>((channel_len >> 24) & 0xFF);
        payload[offset++] = static_cast<std::uint8_t>((channel_len >> 16) & 0xFF);
        payload[offset++] = static_cast<std::uint8_t>((channel_len >> 8) & 0xFF);
        payload[offset++] = static_cast<std::uint8_t>(channel_len & 0xFF);
        std::memcpy(payload.data() + offset, channel.data(), static_cast<std::size_t>(channel_len));
        return payload;
    }
}

    namespace libstp::transport_core
    {
        namespace
        {
            // Spin slice = max wall-clock that spinOnce can sleep when idle.
            // 1 ms gives sub-millisecond p50 dispatch latency on the rrb
            // backend (the iceoryx2 backend it replaced used 10 ms because
            // the underlying state machine was expensive to wake; rrb's
            // poll is a single atomic load so 1 ms is essentially free —
            // 0.1 % of one core at 0 % message rate).
            constexpr int kSharedTransportSpinSliceMs = 1;
        }

        SharedTransport& SharedTransport::instance()
        {
            static SharedTransport shared;
            return shared;
        }

    SharedTransport::SharedTransport()
        : transport_(raccoon::Transport::create())
    {
        // Intentionally no ERROR_MESSAGES auto-subscribe here.
        // Why: every process loading libstp creates this singleton, so
        // an unconditional subscribe meant cli + main.py + vision all
        // opened `raccoon/errors` as subscribers while *nothing* on the
        // host ever published to it. iox2 v0.9.999-dev marks zero-
        // publisher services for destruction on close, and the next
        // process to open hits OpenIsMarkedForDestruction — the exact
        // wedge that defeated nuke+retry self-heal on every restart.
        // Consumers that actually want LCM error broadcasts can call
        // SharedTransport::instance().subscribe_raw(...) themselves.
        ensure_started();
    }

    SharedTransport::~SharedTransport()
    {
        shutdown();
    }

    void SharedTransport::ensure_started()
    {
        // shutdown() destroys the inner Transport's iceoryx2 Node so the
        // singleton can hand it back to a controlled exit path (or to a
        // test that wants a clean slate). Re-create it lazily when the
        // next caller actually wants to use the transport again.
        if (!transport_.is_alive())
        {
            transport_ = raccoon::Transport::create();
        }
        if (spin_daemon_.owns_thread())
        {
            return;
        }

            spin_daemon_ = libstp::threading::ThreadManager::instance().add_daemon(
                "raccoon-transport",
                [this](libstp::threading::stop_token stop)
                {
                    while (!stop.stop_requested())
                    {
                        // Keep the LCM API mutex hold time aligned with raccoon::Transport::spin().
                        // Longer slices can starve concurrent publishers such as the 100 ms watchdog
                        // heartbeat and make messages arrive already stale.
                        transport_.spinOnce(kSharedTransportSpinSliceMs);
                    }
                });
        }

    bool SharedTransport::publish_raw(
        const std::string& channel,
        const void* data,
        int data_len,
        bool reliable,
        bool retained,
        int retry_interval_ms,
        std::uint32_t max_retries)
    {
        ensure_started();
        raccoon::PublishOptions options{};
        options.reliable = reliable;
        options.retained = retained;
        options.retryInterval = std::chrono::milliseconds(retry_interval_ms);
        options.maxRetries = max_retries;
        return transport_.publishRaw(channel, data, data_len, options);
    }

    std::uint64_t SharedTransport::subscribe_raw(
        const std::string& channel,
        RawHandler handler,
        bool reliable,
        bool request_retained)
    {
        ensure_started();
        const auto id = next_subscription_id_.fetch_add(1);
        bool needs_underlying_subscribe = false;
        {
            std::lock_guard lk(mu_);
            auto& handlers = channel_handlers_[channel];
            handlers.emplace(id, std::move(handler));
            subscription_channels_.emplace(id, channel);
            needs_underlying_subscribe = subscribed_channels_.insert(channel).second;
        }

        if (needs_underlying_subscribe)
        {
            raccoon::SubscribeOptions options{};
            options.reliable = reliable;
            options.requestRetained = request_retained;
            const bool ok = transport_.subscribeRaw(
                channel,
                [this, channel](const void* data, int data_len)
                {
                    std::vector<RawHandler> active_handlers;
                    {
                        std::lock_guard lk(mu_);
                        auto it = channel_handlers_.find(channel);
                        if (it == channel_handlers_.end())
                        {
                            return;
                        }
                        active_handlers.reserve(it->second.size());
                        for (const auto& [subscription_id, handler] : it->second)
                        {
                            (void)subscription_id;
                            active_handlers.push_back(handler);
                        }
                    }

                    for (const auto& active_handler : active_handlers)
                    {
                        active_handler(data, data_len);
                    }
                },
                options);

            if (!ok)
            {
                std::lock_guard lk(mu_);
                auto channel_it = channel_handlers_.find(channel);
                if (channel_it != channel_handlers_.end())
                {
                    channel_it->second.erase(id);
                    if (channel_it->second.empty())
                    {
                        channel_handlers_.erase(channel_it);
                    }
                }
                subscription_channels_.erase(id);
                subscribed_channels_.erase(channel);
                return 0;
            }
        }
        else if (request_retained)
        {
            const auto retain_request = encode_retain_request(channel);
            transport_.publishRaw(
                raccoon::Channels::Protocol::RETAIN_REQUEST,
                retain_request.data(),
                static_cast<int>(retain_request.size()));
        }

        return id;
    }

    void SharedTransport::unsubscribe(std::uint64_t subscription_id)
    {
        std::lock_guard lk(mu_);
        auto channel_it = subscription_channels_.find(subscription_id);
        if (channel_it == subscription_channels_.end())
        {
            return;
        }

        auto handlers_it = channel_handlers_.find(channel_it->second);
        if (handlers_it != channel_handlers_.end())
        {
            handlers_it->second.erase(subscription_id);
            if (handlers_it->second.empty())
            {
                channel_handlers_.erase(handlers_it);
            }
        }
        subscription_channels_.erase(channel_it);
    }

    raccoon::TransportStats SharedTransport::get_and_reset_stats()
    {
        ensure_started();
        return transport_.getAndResetStats();
    }

    std::size_t SharedTransport::active_subscription_count() const
    {
        std::lock_guard lk(mu_);
        return subscription_channels_.size();
    }

    void SharedTransport::shutdown()
    {
        // Order matters:
        //   1. Stop+JOIN the spin daemon. Once `spin_daemon_.stop()`
        //      returns, no thread is inside `transport_.spinOnce()`.
        //   2. Call `transport_.shutdown()` to tear down the iceoryx2 Node
        //      and ports HERE — while the calling context (typically a
        //      Python atexit hook) still has iceoryx2's globals alive.
        //      The default ~Transport() would have run during static
        //      teardown and could race iceoryx2's own destructors.
        //   3. Clear our routing tables.
        spin_daemon_.stop();
        transport_.shutdown();
        std::lock_guard lk(mu_);
        channel_handlers_.clear();
        subscription_channels_.clear();
        subscribed_channels_.clear();
    }
}
