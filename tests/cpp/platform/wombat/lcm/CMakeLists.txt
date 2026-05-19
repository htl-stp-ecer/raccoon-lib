find_package(lcm QUIET)

if(lcm_FOUND OR LCM_NAMESPACE)
  register_libstp_test(test_lcm_contract SOURCES test_lcm_reader.cpp
                       test_lcm_writer.cpp LIBS ${LCM_NAMESPACE}lcm)
endif()
