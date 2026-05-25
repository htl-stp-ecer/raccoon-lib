# lcm is provided as an in-tree target by raccoon-transport via FetchContent.
# Avoid find_package() here — it relies on lcmTargets.cmake which is generated
# during CMake's generate phase and isn't reliably available to sub-projects.
if(TARGET lcm)
  register_libstp_test(test_lcm_contract SOURCES test_lcm_reader.cpp
                       test_lcm_writer.cpp LIBS lcm)
endif()
