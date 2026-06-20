register_libstp_test(
  test_motion
  SOURCES
  test_motion_pid.cpp
  test_linear_motion.cpp
  test_directional_line_follow_motion.cpp
  test_turn_motion.cpp
  test_spline_motion.cpp
  test_arc_motion.cpp
  test_diagonal_motion.cpp
  LIBS
  motion
  drive
  kinematics
  odometry)

# test_trapezoidal_profile.cpp is built as a SEPARATE binary, not folded into
# test_motion. The TrapezoidalProfile profile functions (calculate/getSetpoint/
# effectiveDeceleration/validate) are header-defined (inline) and emit weak
# COMDAT definitions in every TU that instantiates them. Test TUs are compiled
# WITHOUT the mull IR-frontend plugin, so a test TU's uninstrumented weak copy
# wins COMDAT folding at link time and SHADOWS the production (instrumented)
# copy — which silently neutralises every trapezoidal_profile.hpp mutant in the
# linked binary (they point at the discarded copies). Keeping the direct unit
# tests in their own executable lets test_motion link the instrumented
# production copy (exercised via ArcMotion/DiagonalMotion ->
# ProfiledPIDController -> TrapezoidalProfile::calculate), so the profile
# mutants are actually scored.
register_libstp_test(
  test_trapezoidal_profile
  SOURCES
  test_trapezoidal_profile.cpp
  LIBS
  motion
  drive
  kinematics
  odometry)
