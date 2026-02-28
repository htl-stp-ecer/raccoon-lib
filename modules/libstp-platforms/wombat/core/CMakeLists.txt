add_library(platform_core STATIC
    src/LcmReader.cpp
    src/LcmWriter.cpp
)

target_link_libraries(platform_core
        PUBLIC
        raccoon::transport
        foundation
)

target_include_directories(platform_core PUBLIC include)
