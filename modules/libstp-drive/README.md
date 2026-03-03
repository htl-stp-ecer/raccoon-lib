# libstp-drive

`libstp-drive` is the chassis-velocity control layer that sits between motion generation and drivetrain kinematics. It accepts desired body-frame velocity, applies per-axis control using encoder and IMU feedback, and hands corrected commands to a `kinematics::IKinematics` implementation.

## Purpose

- Close the loop on chassis velocity without embedding drivetrain geometry into the controller.
- Provide a reusable wrapper around firmware motor velocity control and encoder readback.
- Keep velocity-control tuning local to the drive module while delegating wheel math to `libstp-kinematics`.

This module does not do path planning, pose estimation, or trajectory generation.

## Math and control model

`Drive::update(dt)` currently performs three axis controllers:

- `vx` controller uses `kinematics_->estimateState().vx`
- `vy` controller uses `kinematics_->estimateState().vy`
- `wz` controller uses `imu_.getYawRate()`

Each axis uses the same `VelocityController` structure:

```text
u_ff  = kS * sign(ref) + kV * ref + kA * accel_ref
u_p   = kp * (ref - meas)
u_d   = -kd * filtered_meas_derivative
u_cmd = clamp(u_ff + u_p + ki * integral + u_d, [-u_max, u_max])
```

In the current `Drive` implementation:

- `accel_ref` is always `0.0`
- `u_max` is `infinity`, so the controller is used as a corrective velocity generator rather than an actuator limiter
- the corrected chassis command is passed directly to `IKinematics::applyCommand()`

At the wheel interface, `MotorAdapter::setVelocity()` converts wheel angular velocity in rad/s into firmware BEMF target units using each motor's `ticks_to_rad` calibration and a fixed 200 Hz sampling assumption.

Encoder velocity estimation is also handled in `MotorAdapter`:

- position deltas are converted to rad/s
- implausible jumps are rejected
- the result is low-pass filtered using `vel_lpf_alpha`

## Public C++ surface

Headers:

- `include/drive/drive.hpp`
- `include/drive/velocity_controller.hpp`
- `include/drive/motor_adapter.hpp`

Types and roles:

- `AxisVelocityControlConfig`
  PID plus feedforward terms for one chassis axis.
- `ChassisVelocityControlConfig`
  Per-axis config bundle for `vx`, `vy`, and `wz`.
- `Drive`
  Main façade used by higher layers.
  Key methods:
  - `setVelocity(v_body)`
  - `update(dt)`
  - `estimateState()`
  - `softStop()` / `hardStop()`
  - `setVelocityControlConfig(...)`
  - `resetVelocityControllers()`
  - `getKinematics()`, `getWheelRadius()`, `getMotors()`
- `VelocityController`
  Standalone PID + feedforward helper with derivative filtering and anti-windup.
- `MotorAdapter`
  Thin adapter from `hal::motor::IMotor` to the velocity units expected by the rest of the stack.

## Bindings

Binding module: `libstp.drive`

Exposed Python types:

- `AxisVelocityControlConfig`
- `ChassisVelocityControlConfig`
- `VelocityController`
- `MotorAdapter`
- `Drive`

Binding notes that matter to contributors:

- `Drive.update(dt)` intentionally discards the C++ `MotorCommands` return value and returns `None` in Python.
- `Drive.get_velocity_control_config()` returns a copy.
- `Drive.get_motors()` converts the underlying `IMotor*` list to concrete `hal::motor::Motor*` wrappers.

## Tests

No tests live in this module directory today.

## Extension points

- To support a new drivetrain geometry, add or reuse an `IKinematics` implementation instead of changing `Drive`.
- To change chassis control behavior, extend `VelocityController` or add a sibling controller type, then wire it through `Drive`.
- `Drive` is `final`, so major control-architecture changes should happen through composition or a new module rather than inheritance.
