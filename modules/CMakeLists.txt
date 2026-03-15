function(register_libstp_module module_name is_interface additional_libraries)
    set(CMAKE_CXX_STANDARD 23)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    set(_INC_BUILD "${CMAKE_CURRENT_LIST_DIR}/include")
    set(_INC_INSTALL "include")

    if (is_interface STREQUAL "TRUE")
        message(STATUS "Adding INTERFACE library ${module_name}")
        add_library(${module_name} INTERFACE)
        target_include_directories(${module_name}
                INTERFACE
                $<BUILD_INTERFACE:${_INC_BUILD}>
                $<INSTALL_INTERFACE:${_INC_INSTALL}>
        )
        if (additional_libraries)
            message(STATUS "Link (INTERFACE) deps to ${module_name}: ${additional_libraries}")
            target_link_libraries(${module_name} INTERFACE ${additional_libraries})
        endif ()
    else ()
        file(GLOB_RECURSE MODULE_SRC "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

        if (NOT MODULE_SRC)
            message(WARNING "No sources for ${module_name} in ${CMAKE_CURRENT_LIST_DIR}/src -> creating INTERFACE lib")
            add_library(${module_name} INTERFACE)
            target_include_directories(${module_name}
                    INTERFACE
                    $<BUILD_INTERFACE:${_INC_BUILD}>
                    $<INSTALL_INTERFACE:${_INC_INSTALL}>
            )
            if (additional_libraries)
                target_link_libraries(${module_name} INTERFACE ${additional_libraries})
            endif ()
        else ()
            # Default to STATIC libs to avoid shared-lib cycles across modules.
            set(_lib_type STATIC)
            # Some modules need to be shared so state lives in one place across bindings.
            if (module_name STREQUAL "button" OR module_name STREQUAL "foundation")
                set(_lib_type SHARED)
            endif ()
            add_library(${module_name} ${_lib_type} ${MODULE_SRC})
            target_include_directories(${module_name}
                    PUBLIC
                    $<BUILD_INTERFACE:${_INC_BUILD}>
                    $<INSTALL_INTERFACE:${_INC_INSTALL}>
            )
            if (additional_libraries)
                message(STATUS "Link (PUBLIC) deps to ${module_name}: ${additional_libraries}")
                target_link_libraries(${module_name} PUBLIC ${additional_libraries})
            endif ()
            if (_lib_type STREQUAL "SHARED")
                # Install shared libs alongside Python modules so $ORIGIN works.
                install(TARGETS ${module_name} DESTINATION libstp)
            endif ()
        endif ()
    endif ()

    file(GLOB_RECURSE MODULE_BINDINGS "${CMAKE_CURRENT_LIST_DIR}/bindings/*.cpp")
    if (MODULE_BINDINGS)
        set(PY_LIB_NAME "libstp_${module_name}")
        python_add_library(${PY_LIB_NAME} MODULE
                ${MODULE_BINDINGS}
                WITH_SOABI
        )
        set_target_properties(${PY_LIB_NAME} PROPERTIES OUTPUT_NAME ${module_name})
        # Ensure extension can locate sibling shared libs (e.g., libbutton.so) at runtime.
        set_target_properties(${PY_LIB_NAME} PROPERTIES
                BUILD_RPATH "$ORIGIN"
                INSTALL_RPATH "$ORIGIN")
        target_include_directories(${PY_LIB_NAME}
                PRIVATE
                ${_INC_BUILD}
        )
        target_link_libraries(${PY_LIB_NAME} PRIVATE ${module_name} platform pybind11::headers)
        if (additional_libraries)
            target_link_libraries(${PY_LIB_NAME} PRIVATE ${additional_libraries})
        endif ()
        install(TARGETS ${PY_LIB_NAME} DESTINATION libstp)
    endif ()

    set(_PY_PKG_DIR "${CMAKE_CURRENT_LIST_DIR}/python/libstp")
    if(EXISTS "${_PY_PKG_DIR}")
        message(STATUS "Installing Python package fragment from ${_PY_PKG_DIR}")
        install(
                DIRECTORY "${_PY_PKG_DIR}/"
                DESTINATION libstp
                FILES_MATCHING
                PATTERN "*.py"
                PATTERN "__pycache__" EXCLUDE
        )
    endif()


    message(STATUS "Registered STP module: ${module_name}")
endfunction()

add_subdirectory(libstp-foundation)
add_subdirectory(libstp-hal)
add_subdirectory(libstp-platforms)
add_subdirectory(libstp-async)
add_subdirectory(libstp-kinematics)
add_subdirectory(libstp-drive)
add_subdirectory(libstp-odometry)
add_subdirectory(libstp-robot)
add_subdirectory(libstp-screen)
add_subdirectory(libstp-motion)
add_subdirectory(libstp-sensor-ir)
add_subdirectory(libstp-sensor-et)
add_subdirectory(libstp-button)
add_subdirectory(libstp-mission)
add_subdirectory(libstp-debug)
add_subdirectory(libstp-timing)
add_subdirectory(libstp-servo)
add_subdirectory(libstp-step)
add_subdirectory(libstp-kmeans)
add_subdirectory(libstp-calibration-store)