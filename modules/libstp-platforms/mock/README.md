# mock platform bundle

The `mock` bundle provides an in-process implementation of the HAL for development and tests. It is the easiest bundle to use locally because it does not require raccoon transport, retained topics, or coprocessor firmware.

## What it implements

`mock` registers the following drivers:

- `analog`
- `digital`
- `imu`
- `motor`
- `servo`

Its `platform_core` target is `MockPlatform`, a singleton that stores simulated state for motors, servos, digital inputs, analog inputs, and IMU readings.

## Behavior to know before editing

- Sensor reads are served from in-memory state.
- Analog and IMU reads include small random noise.
- `Motor::setVelocity()` is approximated by mapping the requested value into mock motor state.
- `Motor::moveToPosition()` and `moveRelative()` currently log the request and report completion immediately.
- `IMU::waitForReady()` returns `true` immediately because no asynchronous heading source exists in the mock backend.

This bundle is useful for link verification and higher-level logic tests, but it is not a timing-accurate simulator.

## Testing with the mock bundle

Typical local flow:

```bash
cmake -S . -B build -DDRIVER_BUNDLE=mock
cmake --build build
pytest tests/python
```

If you need deterministic sensor state in C++, use `platform::mock::core::MockPlatform::instance()` and the test helper setters declared in `core/MockPlatform.hpp`.

## Adding a driver to mock

1. Add a new driver subdirectory beside `analog`, `digital`, `imu`, `motor`, and `servo`.
2. Implement the corresponding HAL methods using `MockPlatform` state.
3. Register the new driver in that directory's `CMakeLists.txt`.
4. Add shared simulated state to `MockPlatform` only if more than one driver needs it.
