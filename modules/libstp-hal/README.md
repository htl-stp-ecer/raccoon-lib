# libstp-hal

`libstp-hal` defines the hardware-facing API that the rest of the library consumes. It does two jobs:

- Exposes the public C++ types used by higher-level modules such as `drive`, `odometry`, and `sensor_ir`.
- Builds the `libstp.hal` Python extension from the same API surface.

This module does not contain real hardware drivers. Concrete implementations live in [`modules/libstp-platforms`](../libstp-platforms/README.md) and are selected at configure time through the `platform` target.

## Public API

The headers under `include/hal` are the stable surface platform bundles are expected to implement.

- `AnalogSensor`: integer analog reads from a numbered port.
- `DigitalSensor`: boolean digital reads from a numbered port.
- `Motor` / `IMotor`: percent output, firmware-side velocity control, position moves, BEMF reads, and calibration access.
- `Servo`: set/get position, enable/disable, and global disable.
- `IMU` / `IIMU`: raw accel/gyro/magnetometer reads plus heading, linear acceleration, integrated velocity, and turn-axis helpers.
- `ScreenRender`: singleton screen-state publisher used by the wombat screen driver.

Most constructors and range checks are implemented by the selected platform bundle, not in this module. Shared safety bookkeeping such as duplicate-port tracking and motor fail-safe shutdown lives here so every platform gets the same wrapper behavior.

## Python bindings

If pybind11 is enabled in the overall build, this module also produces a Python extension named `hal` that is installed under the `libstp` package and imported as:

```python
from libstp.hal import AnalogSensor, DigitalSensor, IMU, Motor, Servo
```

Bindings currently exist for:

- `AnalogSensor`
- `DigitalSensor`
- `IMU`
- `IMotor`
- `Motor`
- `Servo`

`ScreenRender` is not bound to Python in this module today.

The binding entrypoint lives in `bindings/bindings.cpp`. It imports `libstp.foundation` first so the `MotorCalibration` type is available before `Motor` is exposed.

## Dependencies

`CMakeLists.txt` registers the module as:

```cmake
register_libstp_module(hal "FALSE" "foundation;raccoon::transport")
```

Practical implications:

- `foundation` supplies logging, shared types, and `MotorCalibration`.
- `raccoon::transport` is linked because `ScreenRender` and some platform implementations depend on raccoon transport types.
- The concrete HAL behavior comes from the separate `platform` target selected in `libstp-platforms`.

## Testing and verification

This module does not define standalone tests under its own directory.

Typical contributor checks are:

1. Configure the build with a concrete platform bundle, usually `-DDRIVER_BUNDLE=mock` for local work.
2. Build the HAL library and bindings.
3. Run the repo-level Python tests that import `libstp.hal` after installation, for example `pytest tests/python`.

The repo's `tests/python/conftest.py` expects `libstp` to be importable and suggests `pip install -e .` for local binding tests.

## How HAL and platforms fit together

At link time, this module expects a target named `platform` to provide the real method definitions for `Motor`, `Servo`, `IMU`, `AnalogSensor`, `DigitalSensor`, and optionally `ScreenRender`. `libstp-platforms` creates that target as an alias to the selected bundle:

- `platform_wombat`
- `platform_mock`

Each bundle compiles sources that implement the methods declared in these headers.

## Adding a new driver capability

When you need to add a new HAL-facing device or extend an existing one:

1. Update or add the public header under `include/hal`.
2. Put wrapper-only logic that should be shared across every platform into `src/`.
3. Add or update the pybind entrypoint under `bindings/` if the API should be exposed to Python.
4. Implement the declared methods in every supported platform bundle in `libstp-platforms`, or document clearly that only a subset of bundles supports the new API.
5. Build at least one real bundle and the `mock` bundle so link errors show up immediately.

## Adding a new platform implementation

If you are not changing the public HAL API, do not edit this module first. Instead:

1. Add a new bundle or driver directory in `libstp-platforms`.
2. Reuse the existing headers in this module.
3. Implement the corresponding methods for that platform.

If the new platform needs bundle-specific helpers, keep those under `libstp-platforms/<bundle>/core` rather than adding them to the public HAL headers.
