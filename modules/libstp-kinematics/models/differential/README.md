# libstp-kinematics differential model

This submodule implements the two-wheel differential-drive realization of `libstp::kinematics::IKinematics`.

## Purpose

- Convert forward and turn-rate commands into left/right wheel angular velocities.
- Recover forward velocity and yaw rate from the measured left/right wheel speeds.
- Provide a model-local place for encoder reset handling.

## Math model

Geometry parameters:

- `wheelbase`: distance between the left and right wheel contact lines, in meters.
- `wheel_radius`: drive wheel radius, in meters.

Inverse kinematics used by `applyCommand()`:

```text
w_left  = (vx - wz * wheelbase / 2) / wheel_radius
w_right = (vx + wz * wheelbase / 2) / wheel_radius
```

Forward kinematics used by `estimateState()`:

```text
vx = (w_left + w_right) * wheel_radius / 2
wz = (w_right - w_left) * wheel_radius / wheelbase
vy = 0
```

The implementation assumes the shared `ChassisVelocity` convention from `libstp-foundation`: `+vx` forward and `+wz` counter-clockwise.

## Public C++ surface

Header: `include/kinematics/differential/differential.hpp`

- `DifferentialKinematics`
  Constructor inputs:
  - left motor pointer
  - right motor pointer
  - `wheelbase`
  - `wheel_radius`

Public behavior:

- `supportsLateralMotion()` always returns `false`.
- `getMotors()` returns `[left, right]` in constructor order.

## Python bindings

Binding module: `libstp.kinematics_differential`

Exposed class:

- `DifferentialKinematics(left_motor, right_motor, wheelbase, wheel_radius)`

The binding also exposes:

- `reset_encoders()`
- `supports_lateral_motion()`
- `get_wheel_radius()`
- `motors`

## Tests

No tests live under this submodule today.

## Extension points

- Keep wheel ordering stable when changing bindings or `getMotors()`, because downstream tooling may index by returned order.
- If you add wheel saturation or limiting inside this model, populate the `MotorCommands` saturation fields so callers can observe it.
