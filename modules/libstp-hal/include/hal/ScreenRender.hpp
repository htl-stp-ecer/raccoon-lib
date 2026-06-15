//
// Created by eternalRose on 12/18/25.
//

#pragma once

#include <string>

namespace libstp::hal::screen_render {
    /**
     * Singleton helper for publishing screen-render state over raccoon transport.
     *
     * This header declares the public surface only. The implementation lives in
     * the selected platform bundle (wombat publishes over raccoon transport;
     * mock is a no-op). The class layout is deliberately transport-free so that
     * every consumer — and the bundle-agnostic ``raccoon.hal`` extension — can
     * include this header without depending on raccoon transport headers. The
     * wombat implementation keeps its ``raccoon::Transport`` instance in a
     * translation-unit-local static inside its ``ScreenRender.cpp`` instead of a
     * member here, so switching the platform bundle at runtime never changes
     * this object's size or layout.
     */
    class ScreenRender {
        std::string screenName;
    public:

        static ScreenRender& instance()
        {
            static ScreenRender writer;
            return writer;
        }

        explicit ScreenRender();
        ~ScreenRender()=default;

        /// Publish the current screen payload if a screen name has been selected.
        void sendState(const std::string& jsonData);

        /// Set the logical screen name used by later `sendState()` calls.
        void setCurrentScreenSetting(std::string newScreen);
    };
}
