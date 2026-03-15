# libstp-kinematics

`libstp-kinematics` defines the chassis-to-wheel conversion layer used by drive control and encoder-based state estimation. It owns the `IKinematics` abstraction plus concrete drivetrain models under [`models/`](./models).

## Purpose

- Convert body-frame velocity commands (`vx`, `vy`, `wz`) into per-wheel angular velocity targets.
- Reconstruct body-frame velocity estimates from motor feedback for drive control and odometry.
- Provide a common integration point for encoder reset behavior.

This module does not plan motion and does not estimate world-frame pose. It stays in chassis space.

## Coordinate and math conventions

- `vx`: forward linear velocity in meters per second.
- `vy`: lateral velocity in meters per second, positive to the robot's right.
- `wz`: angular velocity in radians per second, positive counter-clockwise.
- Wheel velocities are expressed in radians per second.

The shared interface expects each model to implement two transforms:

- Inverse kinematics: chassis velocity to wheel angular velocity.
- Forward kinematics: wheel angular velocity back to chassis velocity.

The concrete equations live in the model submodules:

- [`models/differential`](./models/differential/README.md)
- [`models/mecanum`](./models/mecanum/README.md)

## Public C++ surface

Header: `include/kinematics/kinematics.hpp`

- `MotorCommands`
  Carries the wheel commands produced by `applyCommand()`. The saturation fields exist for callers that need to propagate limit information, although the current in-tree models only fill `wheel_velocities`.
- `IKinematics`
  Abstract contract used by `libstp-drive` and `libstp-odometry`.
  Key methods:
  - `wheelCount()`: number of drive wheels managed by the model.
  - `applyCommand(cmd, dt)`: send a chassis-space command to the underlying motors.
  - `estimateState()`: estimate chassis-space velocity from motor feedback.
  - `hardStop()`: immediately brake the drivetrain.
  - `supportsLateralMotion()`: capability flag for higher layers.
  - `resetEncoders()`: clear encoder tracking after odometry resets.
  - `getWheelRadius()` / `getMotors()`: expose model geometry and owned motors.

## Model submodules

- Differential drive
  Two-wheel skid-steer model with no lateral degree of freedom.
- Mecanum drive
  Four-wheel omnidirectional model with lateral motion support.

Each model wraps raw `hal::motor::IMotor` instances with `drive::MotorAdapter`, so encoder filtering and command conversion are shared with `libstp-drive`.

## Python bindings

Binding module: `libstp.kinematics`

Exposed type:

- `IKinematics`
  Methods mirror the C++ base interface for `wheel_count`, `apply_command`, `estimate_state`, and `hard_stop`.
  The `motors` property returns the underlying motor objects.

Concrete Python-constructible models are exported by the model submodules, not by this base module.

## Tests

No unit tests or module-local integration tests live in this directory tree today. Contributors changing kinematic math should expect to verify behavior through downstream drive or odometry integration.

## Extension points

- Add a new drivetrain model by subclassing `IKinematics` and placing it under `models/<name>/`.
- Keep the body-frame sign conventions above unchanged; drive and odometry assume them.
- If a new model needs additional command metadata, extend `MotorCommands` without breaking existing callers that only consume `wheel_velocities`.
