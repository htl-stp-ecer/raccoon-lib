function(register_libstp_module module_name is_interface additional_libraries)
    # Explicitly check if is_interface is TRUE
    if(is_interface STREQUAL "TRUE")
        message("Adding interface library ${module_name} ${is_interface} ${additional_libraries}")
        add_library(${module_name} INTERFACE)
        target_include_directories(${module_name} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
        if(additional_libraries)
            target_link_libraries(${module_name} INTERFACE ${additional_libraries})
        endif()
    else()
        file(GLOB_RECURSE MODULE_SRC "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
        
        if(NOT MODULE_SRC)
            message(WARNING "No source files found for module '${module_name}' in ${CMAKE_CURRENT_LIST_DIR}/src")
            set(MODULE_SRC "") 
        endif()

        add_library(${module_name} STATIC ${MODULE_SRC})
        target_include_directories(${module_name} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
        if(additional_libraries)
            target_link_libraries(${module_name} PUBLIC ${additional_libraries})
        endif()
    endif()

    file(GLOB_RECURSE MODULE_BINDINGS "${CMAKE_CURRENT_LIST_DIR}/bindings/*.cpp")
    
    if(MODULE_BINDINGS)
        set(PY_LIB_NAME "libstp_${module_name}")
        python_add_library(${PY_LIB_NAME} MODULE
                ${MODULE_BINDINGS}
                ${MODULE_SRC}
                WITH_SOABI
        )
        set_target_properties(${PY_LIB_NAME} PROPERTIES OUTPUT_NAME ${module_name})
        target_include_directories(${PY_LIB_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
        target_link_libraries(${PY_LIB_NAME} PRIVATE ${module_name} platform pybind11::headers)
        if(additional_libraries)
            target_link_libraries(${PY_LIB_NAME} PRIVATE ${additional_libraries})
        endif()
        install(TARGETS ${PY_LIB_NAME} DESTINATION libstp)
    endif()

    MESSAGE(STATUS "Adding ${module_name} ${CMAKE_CURRENT_LIST_DIR}")
    MESSAGE(STATUS "Registering STP module: ${module_name} with sources: ${MODULE_SRC}, bindings: ${MODULE_BINDINGS}, and additional libraries: ${additional_libraries}")
endfunction()

add_subdirectory(libstp-foundation)
add_subdirectory(libstp-hal)
add_subdirectory(libstp-platforms)
add_subdirectory(libstp-async)
add_subdirectory(libstp-sensors)
add_subdirectory(libstp-motion)
add_subdirectory(libstp-datatype)
add_subdirectory(libstp-ahrs)
#add_subdirectory(libstp-drives) # ToDo: This is a sample refactor that will come once all is working
add_subdirectory(libstp-device)
