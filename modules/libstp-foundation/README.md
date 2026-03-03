# libstp-foundation

Contributor reference for the low-level types, control helpers, math utilities, and logging
infrastructure shared across the rest of `libstp`.

## Purpose

`libstp-foundation` is the common dependency for modules that need:

- shared Eigen-backed data types such as `Pose` and `ChassisVelocity`
- reusable controller/configuration types such as `PidController` and feedforward helpers
- small math utilities used across motion and estimation code
- process-wide logging with runtime filtering and Python access

The module is built as a shared library so logging state is not duplicated across Python
extensions.

## Architecture

The public surface is split by concern:

- [`include/foundation/types.hpp`](include/foundation/types.hpp): common geometric and unit-like
  types
- [`include/foundation/motor.hpp`](include/foundation/motor.hpp): feedforward and motor-related
  configuration structs
- [`include/foundation/pid.hpp`](include/foundation/pid.hpp): PID controller implementation and
  tuning parameters
- [`include/foundation/math.hpp`](include/foundation/math.hpp): free utility functions
- [`include/foundation/logging.hpp`](include/foundation/logging.hpp): process-wide logger API and
  macros
- [`include/foundation/pch.hpp`](include/foundation/pch.hpp): Eigen precompiled header umbrella
- [`include/foundation/config.hpp`](include/foundation/config.hpp): convenience include used by
  bindings

Implementation lives in `src/`:

- `logging.cpp` configures a single `spdlog` logger with console and rotating-file sinks
- `pid.cpp` owns PID state and filtering
- `math.cpp` implements the free functions declared in the header

Python bindings live in `bindings/` and expose a subset of the C++ API.

## Public API

The main C++ entrypoints are:

- `logging::init()`, `logging::initialize_timer()`, `set_global_level()`, `set_file_level()`, and the
  `LIBSTP_LOG_*` macros
- `libstp::foundation::PidConfig` and `PidController`
- `libstp::foundation::PidGains`, `Feedforward`, `MotorCalibration`, and
  `FeedforwardController`
- `libstp::foundation::Pose` and `ChassisVelocity`
- `libstp::foundation::Meters`, `MetersPerSecond`, `Radians`, and `RadiansPerSecond`
- `libstp::math::lerp()`, `easeInOut()`, `clampf()`, `clampDouble()`, `clampInt()`, `sign()`,
  `signf()`, `minimalAngleDifference()`, and `quaternionToEuler()`

Notes for contributors:

- `logging` intentionally lives in the plain `logging` namespace, not under `libstp`.
- `PidController::update(error, dt)` differentiates the error signal; the overload taking
  `deriv_signal` lets callers supply a measurement-derived derivative instead.
- The unit wrapper types are lightweight value objects with conversion helpers and literals, not a
  full dimensional-analysis system.

## Dependencies

Build-time and link-time dependencies declared by this module:

- `Eigen3::Eigen`
- `spdlog::spdlog`

The Python extension additionally depends on `pybind11` and the project `platform` target through
the shared module registration helper.

## Python Bindings

`bindings/bindings.cpp` builds a Python extension module named `foundation`.

Currently exposed bindings:

- `PidConfig`, `PidController`
- `PidGains`, `Feedforward`, `MotorCalibration`, `FeedforwardController`
- `Pose`, `ChassisVelocity`
- `Level` plus logging helpers such as `initialize_logging()`, `set_global_level()`, and
  `_log_filtered()`

Not currently exposed:

- the math free functions
- the strongly-typed unit wrappers in `types.hpp`

If you add bindings, keep the C++/Python naming aligned with the existing module instead of
creating parallel convenience types.

## Testing

No module-local tests or benchmarks exist in this directory today. Validation currently depends on
downstream consumers building and exercising the module.

Before changing behavior, at minimum:

- build the C++ target and Python extension
- import `foundation` from Python if you touched bindings
- exercise PID and logging behavior from a small focused harness if semantics changed

## Extension Points

Common contributor additions here include:

- new shared value types in `types.hpp`
- additional controller helpers beside PID/feedforward
- extra math utilities that are broadly reusable
- more Python bindings for already-public C++ types

Keep this module conservative. If an API is domain-specific to one subsystem, prefer placing it in
that subsystem and depending on `foundation` rather than growing `foundation` itself.
