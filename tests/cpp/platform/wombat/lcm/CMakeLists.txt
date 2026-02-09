register_libstp_test(test_lcm
        SOURCES
        test_lcm_reader.cpp
        test_lcm_writer.cpp
        LIBS
        platform
        ${LCM_NAMESPACE}lcm
)
