//
// Created by eternalRose on 12/18/25.
//

#ifndef VERSION_SCREENRENDER_HPP
#define VERSION_SCREENRENDER_HPP
#include <string>

#include <raccoon/Transport.h>

#endif //VERSION_SCREENRENDER_HPP


namespace libstp::hal::screen_render {
    /**
     * Singleton helper for publishing screen-render state over raccoon transport.
     *
     * This header declares the public surface; the in-tree implementation exists
     * only for the wombat platform bundle.
     */
    class ScreenRender {
        std::string screenName;
        raccoon::Transport transport_;
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
