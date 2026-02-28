register_driver(screen_driver)

target_link_libraries(screen_driver
        PUBLIC
        raccoon::transport
        foundation
)

target_include_directories(screen_driver PUBLIC include)
