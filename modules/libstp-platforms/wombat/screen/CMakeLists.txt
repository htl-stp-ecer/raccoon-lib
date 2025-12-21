register_driver(screen_driver)

target_link_libraries(screen_driver
        PUBLIC
        messages-cpp
        ${LCM_NAMESPACE}lcm
        foundation
)

target_include_directories(screen_driver PUBLIC include)
target_link_libraries(screen_driver PUBLIC messages-cpp ${LCM_NAMESPACE}lcm)