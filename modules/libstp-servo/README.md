# libstp-servo

`libstp-servo` is a Python-only helper module that adds servo-focused steps to the shared `libstp.step` namespace.

Unlike most modules in this repo, it does not ship a native C++ library or pybind module. It builds on the HAL `Servo` type provided by `libstp-hal` and contributes ergonomics for mission authors.

## Public API

Python package:

- `python/libstp/step/servo/`

Main exports:

- `SetServoPosition`
- `EaseServo`
- `ShakeServo`
- `servo(...)`
- `slow_servo(...)`
- `shake_servo(...)`
- `resolve_servo(...)`
- `angle_to_position(...)`
- `position_to_angle(...)`
- `estimate_servo_move_time(...)`

## Responsibilities

- Wrap one-shot servo moves in step objects.
- Provide conservative timing estimates so sequences wait long enough for a move to finish.
- Offer a smoother eased move and a repeated shake helper.
- Resolve servo references either by object instance or by attribute name on `robot.defs`.

## Conventions

- Servo angles are expressed in degrees.
- The helper range is constrained by `SERVO_MIN_ANGLE` and `SERVO_MAX_ANGLE`.
- `angle_to_position()` converts degrees into the HAL position scale expected by the concrete servo implementation.
- Timing helpers are estimates only; they should be treated as workflow-friendly delays, not as hardware truth.

## Extension Points

- Keep hardware-specific behavior in `libstp-hal`; this module should stay focused on step ergonomics.
- If you add new movement patterns, build them from the existing conversion and resolution helpers.
- Update the constants and README together if the expected HAL position scale changes.
