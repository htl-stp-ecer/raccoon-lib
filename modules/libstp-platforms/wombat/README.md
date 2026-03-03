# wombat platform bundle

The `wombat` bundle connects the HAL wrappers to the Wombat transport and firmware protocol. It is the default `DRIVER_BUNDLE` in this repository.

## What it implements

`wombat` registers these drivers:

- `analog_driver`
- `digital_driver`
- `imu_driver`
- `motor_driver`
- `servo_driver`
- `screen_driver`

`platform_core` for this bundle consists of:

- `LcmReader`: subscribes to raccoon channels, caches the latest sensor and status data, and exposes synchronous reads to HAL drivers.
- `LcmDataWriter`: publishes motor, servo, PID, reset, and shutdown commands.

## Transport model

The bundle uses `raccoon::Transport::create()` in both reader and writer helpers.

- `LcmReader` starts a background listener thread on construction.
- High-rate sensor topics such as gyro, accel, and magnetometer are cached as messages arrive.
- Several control/status topics use retained subscriptions so newly started processes can read the latest known state.
- HAL drivers stay synchronous because they only talk to the reader/writer caches.

For contributors, this means most Wombat changes belong in `core/` first and only secondarily in the individual device wrappers.

## Public behaviors that matter

- `Motor::setSpeed()` publishes percent output directly, with inversion applied in software.
- `Motor::setVelocity()`, `moveToPosition()`, and `moveRelative()` delegate to firmware-side control channels.
- `Motor::disableAll()` asserts the STM32 shutdown flag and also sends per-port stop commands.
- `IMU::getAngularVelocity()` converts incoming gyro degrees/sec to radians/sec.
- `IMU::getHeading()` converts heading degrees to radians and flips sign to match libstp's CCW-positive convention.
- `ScreenRender` publishes only when a screen name has been set through `setCurrentScreenSetting()`.

## Dependencies

`wombat/core` links:

- `raccoon::transport`
- `foundation`

`wombat/screen` also links:

- `raccoon::transport`
- `foundation`

The rest of the drivers link through `register_driver(...)`, which already adds `hal` and `platform_core`.

## Testing and verification

This bundle does not include standalone tests under its own directory. Practical checks are:

1. Build with `-DDRIVER_BUNDLE=wombat`.
2. Smoke-test the `libstp.hal` bindings or a C++ consumer against a running transport stack.
3. If you changed reader cache semantics, verify startup behavior where retained topics may be absent.

For logic-only changes, validate against the `mock` bundle first and then rebuild `wombat` to catch transport-specific compile or link issues.

## Adding a new driver or capability

1. Confirm the public HAL declaration exists in `libstp-hal`.
2. Decide whether the new behavior is command-oriented, read-oriented, or both.
3. Put shared transport/channel logic into `core/LcmReader.*` or `core/LcmWriter.*`.
4. Implement the device-facing HAL methods in a new or existing driver directory.
5. Register the driver target from the new directory's `CMakeLists.txt`.

If the new feature is Wombat-only, document that clearly in the HAL README or bundle README instead of implying every platform supports it.
