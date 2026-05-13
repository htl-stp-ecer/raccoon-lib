#include "hal/ScreenRender.hpp"

libstp::hal::screen_render::ScreenRender::ScreenRender()
#ifndef DRIVER_BUNDLE_MOCK
    : transport_()
#endif
{
    // Mock: no LCM transport — sendState is a no-op.
}

void libstp::hal::screen_render::ScreenRender::setCurrentScreenSetting(std::string newScreen)
{
    screenName = std::move(newScreen);
}

void libstp::hal::screen_render::ScreenRender::sendState(const std::string& jsonData)
{
    // no-op on mock — no raccoon transport publishing
}
