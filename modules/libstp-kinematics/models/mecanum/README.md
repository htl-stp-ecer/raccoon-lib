# libstp-kinematics mecanum model

This submodule implements the four-wheel mecanum realization of `libstp::kinematics::IKinematics`.

## Purpose

- Convert body-frame translational and rotational commands into four wheel angular velocities.
- Recover chassis velocity from the measured wheel speeds.
- Centralize encoder reset behavior for a mecanum drivetrain.

## Math model

Geometry parameters:

- `wheelbase`: front-to-back axle spacing in meters.
- `track_width`: left-to-right wheel spacing in meters.
- `wheel_radius`: drive wheel radius in meters.

The implementation defines:

```text
L = (wheelbase + track_width) / 2
```

Wheel order is front-left, front-right, back-left, back-right.

Inverse kinematics used by `applyCommand()`:

```text
w_fl = (vx + vy - L * wz) / wheel_radius
w_fr = (vx - vy + L * wz) / wheel_radius
w_bl = (vx - vy - L * wz) / wheel_radius
w_br = (vx + vy + L * wz) / wheel_radius
```

Forward kinematics used by `estimateState()`:

```text
vx = (w_fl + w_fr + w_bl + w_br) * wheel_radius / 4
vy = (w_fl - w_fr - w_bl + w_br) * wheel_radius / 4
wz = (-w_fl + w_fr - w_bl + w_br) * wheel_radius / (4 * L)
```

The body-frame convention matches the rest of LibSTP: `+x` forward, `+y` right, `+wz` counter-clockwise.

## Public C++ surface

Header: `include/kinematics/mecanum/mecanum.hpp`

- `MecanumKinematics`
  Constructor inputs:
  - front-left motor
  - front-right motor
  - back-left motor
  - back-right motor
  - `wheelbase`
  - `track_width`
  - `wheel_radius`

Public behavior:

- `supportsLateralMotion()` always returns `true`.
- `getMotors()` preserves the same front-left, front-right, back-left, back-right ordering.

## Python bindings

Binding module: `libstp.kinematics_mecanum`

Exposed class:

- `MecanumKinematics(front_left_motor, front_right_motor, back_left_motor, back_right_motor, wheelbase, track_width, wheel_radius)`

Additional helpers exposed to Python:

- `reset_encoders()`
- `supports_lateral_motion()`
- `get_wheel_radius()`
- `motors`

## Tests

No tests live under this submodule today.

## Extension points

- Preserve wheel ordering across bindings, docs, and implementation. The equations and returned motor list depend on the same order.
- If you introduce alternate roller geometry or sign conventions, document them explicitly because downstream drive and odometry code assume the current frame conventions.
