register_libstp_test(
  test_localization
  SOURCES
  test_localization.cpp
  LIBS
  localization
  odometry
  foundation)

register_libstp_test(
  test_localization_edge_cases
  SOURCES
  test_localization_edge_cases.cpp
  LIBS
  localization
  odometry
  foundation)

register_libstp_test(
  test_heartbeat_coexistence
  SOURCES
  test_heartbeat_coexistence.cpp
  LIBS
  localization
  threading
  odometry
  foundation)
