#include "transport_core/shared_transport.hpp"

#include "foundation/logging.hpp"

namespace
{
    // Route the transport's reliable-delivery events into libstp's structured log instead of
    // the transport's cerr fallback: a WARN on the first retransmit (a command was lost on the
    // first send but is being recovered) and an ERROR on give-up (an unrecovered command loss).
    void install_reliable_logging(raccoon::Transport& transport)
    {
        transport.set_reliable_event_handler(
            [](raccoon::Transport::ReliableEvent event, const std::string& channel,
               std::int64_t key, std::uint32_t attempts)
            {
                if (event == raccoon::Transport::ReliableEvent::GaveUp)
                {
                    LIBSTP_LOG_ERROR(
                        "reliable command LOST on {} (ts={}) — no ACK after {} "
                        "sends; the subscriber never received it",
                        channel, key, attempts);
                }
                else
                {
                    LIBSTP_LOG_WARN(
                        "reliable command on {} (ts={}) not acked — "
                        "retransmitting (attempt {})",
                        channel, key, attempts);
                }
            });
    }
}

namespace libstp::transport_core
{
    SharedTransport& SharedTransport::instance()
    {
        static SharedTransport shared;
        return shared;
    }

    SharedTransport::~SharedTransport()
    {
        shutdown();
    }

    raccoon::Transport& SharedTransport::started()
    {
        // shutdown() stops the transport so that a controlled exit (or a test) can end it
        // while the process is whole. The next caller gets a new one.
        std::lock_guard<std::mutex> lk(mu_);
        if (!started_ || !transport_.is_alive())
        {
            started_ = true;
            transport_ = raccoon::Transport::create();
            install_reliable_logging(transport_);
            transport_.start_background_spin();
        }
        return transport_;
    }

    raccoon::PublishResult SharedTransport::publish_raw(std::string_view channel, const void* data,
                                                        std::size_t len,
                                                        const raccoon::PublishOptions& options)
    {
        return started().publish_raw(channel, data, len, options);
    }

    SharedTransport::SubscriptionId SharedTransport::subscribe_raw(std::string_view channel,
                                                                   RawHandler handler)
    {
        return started().subscribe_raw(channel, std::move(handler));
    }

    void SharedTransport::unsubscribe(SubscriptionId id)
    {
        started().unsubscribe(id);
    }

    raccoon::TransportStats SharedTransport::get_and_reset_stats()
    {
        return started().get_and_reset_stats();
    }

    void SharedTransport::shutdown()
    {
        // Joins the transport thread; the handlers it runs must not wait for this thread.
        transport_.shutdown();
        started_ = false;
    }
}
