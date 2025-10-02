add_library(platform_core STATIC
        src/LcmReader.cpp
        src/LcmWriter.cpp
)

target_include_directories(platform_core PUBLIC include)
target_link_libraries(platform_core PUBLIC messages-cpp ${LCM_NAMESPACE}lcm)