# libstp-motion

`libstp-motion` contains the closed-loop motion primitives used by the rest of the robot stack. It sits above `libstp-drive` and `libstp-odometry`: drive turns chassis velocity requests into wheel commands, odometry estimates what the robot actually did, and motion closes the loop around both.

This module has two public layers:

- A C++ runtime layer for reusable motion controllers such as straight-line, diagonal, and turn motions.
- A Python step layer under `libstp.step.motion` that wraps those controllers into the async step DSL used by missions.

## Responsibilities

- Generate smooth setpoints with trapezoidal motion profiles.
- Track those setpoints with profiled PID control.
- Correct heading and cross-track drift while a motion is running.
- Expose telemetry so tuning and characterization steps can inspect controller behavior.
- Provide higher-level Python steps for drive, turn, strafe, line following, lineup, wall alignment, and tuning workflows.

## Architecture

The C++ API is built around `MotionContext` and `Motion`:

- `MotionContext` bundles the `Drive`, `IOdometry`, and `UnifiedMotionPidConfig` references that every primitive needs.
- `Motion` is the abstract lifecycle interface: `start()`, `update(dt)`, and `isFinished()`.
- Concrete controllers implement a specific motion type:
  - `LinearMotion` for forward/backward and lateral translation.
  - `DiagonalMotion` for travel along an arbitrary body-frame angle.
  - `TurnMotion` for in-place heading changes.

The Python layer mirrors that structure:

- `MotionStep` owns the fixed-rate async loop and lifecycle hooks.
- Small wrapper steps such as `Drive`, `Turn`, and `DriveAngle` instantiate the C++ controllers.
- Higher-level helper packages such as `lineup`, `line_follow`, `tune_drive`, `characterize_drive`, and `auto_tune` compose those wrappers into contributor-facing DSL helpers.

## Public C++ API

Primary headers:

- `include/motion/motion.hpp`
- `include/motion/motion_config.hpp`
- `include/motion/linear_motion.hpp`
- `include/motion/diagonal_motion.hpp`
- `include/motion/turn_motion.hpp`
- `include/motion/profiled_pid_controller.hpp`
- `include/motion/trapezoidal_profile.hpp`
- `include/motion/motion_pid.hpp`

Important types:

- `MotionContext`
- `Motion`
- `AxisConstraints`
- `UnifiedMotionPidConfig`
- `LinearMotionConfig`
- `LinearMotionTelemetry`
- `DiagonalMotionConfig`
- `DiagonalMotionTelemetry`
- `TurnConfig`

Coordinate conventions:

- Forward motion is positive `vx`.
- Rightward lateral motion is positive `vy`.
- Counter-clockwise rotation is positive `wz`.
- `TurnConfig.target_angle_rad > 0` means a left / counter-clockwise turn.
- `DiagonalMotionConfig.angle_rad == 0` means forward travel, `pi/2` means right strafe, and `-pi/2` means left strafe.

## Public Python API

The Python namespace is split between the native `libstp.motion` module and pure-Python step helpers:

- Native bindings expose the C++ controller classes and config/telemetry datatypes.
- `libstp.step.motion` exports the mission-facing step DSL.

Main step entrypoints:

- `drive_forward`, `drive_backward`, `strafe_left`, `strafe_right`
- `turn_left`, `turn_right`
- `drive_angle`
- `stop`
- `follow_line`, `follow_line_single`
- `lineup(...)` and the specialized lineup helpers
- `wall_align_*`
- `wait_until_distance`
- `tune_drive`
- `characterize_drive`
- `auto_tune`, `auto_tune_velocity`, `auto_tune_motion`

## Configuration And Tuning

`UnifiedMotionPidConfig` is the main contributor-facing config object. It combines:

- Position / distance PID tuning.
- Heading PID tuning.
- Feedforward weight for the profiled controller.
- Saturation derating and recovery parameters.
- Motion completion tolerances.
- Per-axis constraints for linear, lateral, and angular motion.

Those axis constraints are expected to come from measured robot behavior, not from arbitrary guesses. In this repo the intended workflow is:

1. Characterize the drivetrain with `characterize_drive`.
2. Store the measured maximum velocity, acceleration, and deceleration.
3. Tune the motion loops with `tune_drive` or `auto_tune*` if needed.
4. Feed the resulting values into `robot.motion_pid_config`.

## Extension Points

When adding a new C++ motion primitive:

1. Add a new config type and controller header under `include/motion/`.
2. Implement the controller in `src/`.
3. Bind it in `bindings/` if it must be visible from Python.
4. Add a Python `MotionStep` wrapper if missions should use it directly.
5. Add tests for convergence, saturation behavior, and completion conditions.

When adding a new Python helper:

1. Prefer composing existing motion controllers before writing a new low-level loop.
2. Keep the helper in `python/libstp/step/motion/` close to similar functionality.
3. Give the step a stable `_generate_signature()` if timing metrics should stay comparable.
4. Update `python/libstp/step/motion/__init__.py` so contributors can discover it.

## Testing

Relevant tests live in:

- `tests/cpp/motion/`
- `tests/python/test_motion_bindings.py`

The C++ tests cover controller behavior and profile math. The Python tests cover binding visibility and step-level integration.

## Known Constraints

- Motion controllers assume they own the drive command for the duration of the step.
- `update(dt)` must be called at a stable cadence; the Python wrappers default to a 100 Hz loop.
- Heading correction depends on odometry quality. Poor IMU alignment or stale odometry will show up as path drift.
- Some Python helpers depend on other modules, especially `libstp-step`, `libstp-screen`, and `libstp-sensor-ir`.
