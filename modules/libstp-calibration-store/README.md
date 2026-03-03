# libstp-calibration-store

Contributor reference for the YAML-backed singleton used to persist simple calibration readings
across runs.

## Purpose

`libstp-calibration-store` stores and retrieves calibration thresholds keyed by
`CalibrationType`. The current implementation is intentionally narrow and is used for persisting a
small amount of sensor calibration data rather than for general configuration management.

Today the module supports one calibration family:

- `IR_SENSOR`

## Architecture

The module has three parts:

- [`include/CalibrationType.hpp`](include/CalibrationType.hpp): enum identifying supported
  calibration families
- [`include/CalibrationStore.hpp`](include/CalibrationStore.hpp): singleton access and storage API
- [`src/CalibrationStore.cpp`](src/CalibrationStore.cpp): YAML file I/O and type-to-key mapping

Python bindings live in `bindings/` and expose free functions that delegate to the singleton.

Implementation details that matter to contributors:

- the backing file path is hard-coded as `./racoon.calibration.yml`
- data is written under the YAML key `root`
- `IR_SENSOR` maps to the YAML key `ir-calibration`
- stored keys are `white_tresh` and `black_tresh` as spelled in the current implementation
- missing files or missing nodes read back as `{0.0, 0.0}` for `getReadings()` and `false` for
  `hasReadings()`

## Public API

The public C++ API is:

- `enum libstp::calibration_store::CalibrationType`
- `class libstp::calibration_store::CalibrationStore`
- `CalibrationStore::instance()`
- `CalibrationStore::doesFileExist() const`
- `CalibrationStore::storeReading(float black_tresh, float white_tresh, CalibrationType type) const`
- `CalibrationStore::hasReadings(CalibrationType type) const`
- `CalibrationStore::getReadings(CalibrationType type) const`

`CalibrationStore` is deliberately non-copyable and non-movable. All access is expected to go
through `instance()`.

## Dependencies

Declared dependencies for this module:

- `foundation`
- `yaml-cpp`

The implementation uses `foundation` only for logging at the moment.

## Python Bindings

`bindings/bindings.cpp` builds a Python extension module named `calibration_store`.

Currently exposed bindings:

- `CalibrationType`
- `store_readings(...)`
- `get_readings(type)`
- `has_readings(type)`

Binding note:

- `store_readings(type, whiteThreshold, blackThreshold)` now matches the advertised Python
  signature. Internally it still forwards into the C++ storage API, which stores
  `(black_tresh, white_tresh, type)`.

## Testing

No module-local tests exist in this directory.

If you change the store, verify at least:

- first-write behavior when the YAML file does not exist yet
- overwrite behavior for an existing calibration type
- malformed or empty files if you add error handling
- Python binding calls, especially for `store_readings`

## Extension Points

The current implementation is easy to extend in a few specific directions:

- add new `CalibrationType` values and update the type-to-YAML-key switch statements
- make the file path configurable instead of hard-coded
- add validation/error reporting for malformed YAML
- expose a richer Python API once the argument ordering issue is addressed in code

If you add a new calibration family, update both the enum and every `switch` in
`CalibrationStore.cpp`; unsupported values currently fall back to `"unknown-calibration"` on write
and `false`/`{0.0, 0.0}` on read.
