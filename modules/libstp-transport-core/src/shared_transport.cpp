#include "transport_core/shared_transport.hpp"

#include <chrono>

namespace libstp::transport_core
{
    SharedTransport& SharedTransport::instance()
    {
        static SharedTransport shared;
        return shared;
    }

    SharedTransport::SharedTransport()
        : transport_(raccoon::Transport::create())
    {
        ensure_started();
    }

    SharedTransport::~SharedTransport()
    {
        shutdown();
    }

    void SharedTransport::ensure_started()
    {
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
                    transport_.spinOnce(50);
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
        {
            std::lock_guard lk(mu_);
            active_subscriptions_.insert(id);
            handlers_.emplace(id, std::move(handler));
        }

        raccoon::SubscribeOptions options{};
        options.reliable = reliable;
        options.requestRetained = request_retained;
        transport_.subscribeRaw(
            channel,
            [this, id](const void* data, int data_len)
            {
                RawHandler active_handler;
                {
                    std::lock_guard lk(mu_);
                    if (!active_subscriptions_.contains(id))
                    {
                        return;
                    }
                    auto it = handlers_.find(id);
                    if (it == handlers_.end())
                    {
                        return;
                    }
                    active_handler = it->second;
                }
                active_handler(data, data_len);
            },
            options);

        return id;
    }

    void SharedTransport::unsubscribe(std::uint64_t subscription_id)
    {
        std::lock_guard lk(mu_);
        active_subscriptions_.erase(subscription_id);
        handlers_.erase(subscription_id);
    }

    void SharedTransport::shutdown()
    {
        spin_daemon_.stop();
        transport_.stop();
        std::lock_guard lk(mu_);
        active_subscriptions_.clear();
        handlers_.clear();
    }
}
