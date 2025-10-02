add_library(platform_core STATIC
        src/MockPlatform.cpp
)

target_include_directories(platform_core PUBLIC include)