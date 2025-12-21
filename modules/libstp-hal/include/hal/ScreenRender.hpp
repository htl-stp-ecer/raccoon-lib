//
// Created by eternalRose on 12/18/25.
//

#ifndef VERSION_SCREENRENDER_HPP
#define VERSION_SCREENRENDER_HPP
#include <string>

#include "lcm/lcm-cpp.hpp"

#endif //VERSION_SCREENRENDER_HPP


namespace libstp::hal::screen_render {
    class ScreenRender {
        std::string screenName;
        lcm::LCM lcm;
    public:

        static ScreenRender& instance()
        {
            static ScreenRender writer;
            return writer;
        }

        explicit ScreenRender();
        ~ScreenRender()=default;

        void sendState(const std::string& jsonData);

        void setCurrentScreenSetting(std::string newScreen);
    };
}