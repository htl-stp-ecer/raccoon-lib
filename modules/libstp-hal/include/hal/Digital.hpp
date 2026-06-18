#pragma once

#include <memory>
#include <vector>

#include "hal/PortRegistry.hpp"

namespace libstp::hal::digital
{
    /**
     * Digital input wrapper implemented by the active platform bundle.
     *
     * The wrapper only owns the requested port number and shared safety-check
     * bookkeeping. Platform drivers provide the actual I/O behavior.
     *
     * `read()` is virtual so composite inputs (see ButtonGroup) can override
     * the sensing logic while still being usable anywhere a DigitalSensor is.
     */
    class DigitalSensor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline detail::PortRegistry registry_{};
        static void registerDigitalPort(int port);
        static void unregisterDigitalPort(int port);
#endif
    public:
        int port;

        /// Create a digital input bound to a platform-defined port index.
        explicit DigitalSensor(int port);

        virtual ~DigitalSensor();

        /// Return the current boolean state for the configured input.
        virtual bool read() const;
    };

    /**
     * Composite digital input that reads pressed when ANY member port reads
     * pressed (logical OR).
     *
     * It IS-A DigitalSensor, so it can be used anywhere a single DigitalSensor
     * is expected — in particular as the robot's primary button, letting
     * several physical buttons act as one trigger. The inherited `port` is the
     * first member port.
     */
    class ButtonGroup final : public DigitalSensor
    {
    public:
        /// Construct from one or more platform port indices (must be non-empty).
        explicit ButtonGroup(std::vector<int> ports);

        /// True when any member port reads pressed.
        bool read() const override;

        /// The configured member ports.
        const std::vector<int>& ports() const { return ports_; }

    private:
        std::vector<int> ports_;
        // Owns sensors for ports_[1..]; ports_[0] is the inherited base.
        std::vector<std::unique_ptr<DigitalSensor>> extra_;
    };
}
