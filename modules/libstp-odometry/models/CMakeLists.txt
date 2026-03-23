add_subdirectory(fused)
if(DRIVER_BUNDLE STREQUAL "wombat")
    add_subdirectory(stm32)
endif()
