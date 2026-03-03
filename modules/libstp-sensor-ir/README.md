# libstp-sensor-ir

`libstp-sensor-ir` provides the reflectance-sensor abstraction used for line sensing, line following, and calibration workflows.

The module wraps a HAL analog input, stores black/white thresholds, and exposes helper methods for classification and simple probabilistic scoring. It also contains a small multi-sensor calibration helper used by higher-level setup flows.

## Public API

Primary headers:

- `include/IRSensor.hpp`
- `include/IRSensorCalibration.hpp`

Python bindings:

- native module `libstp.sensor_ir`

Main classes:

- `IRSensor`
- `IRSensorCalibration`

## Runtime Model

An `IRSensor` is an `AnalogSensor` with extra state:

- `whiteThreshold`
- `blackThreshold`
- `whiteMean`
- `blackMean`
- `whiteStdDev`
- `blackStdDev`

At runtime, contributors typically:

1. Construct the sensor on a HAL analog port.
2. Calibrate it from sampled values or restore previously stored thresholds.
3. Query `isOnBlack()`, `isOnWhite()`, or the probability helpers while driving.

## Calibration

`IRSensor::calibrate(values)` derives thresholds from a batch of readings. `IRSensorCalibration` is the batch helper for workflows that collect readings across multiple sensors.

This module depends on other infrastructure for a full user-facing calibration experience:

- threshold persistence in `libstp-calibration-store`
- user interaction in `libstp-step` / `libstp-screen`
- button gating in `libstp-button`

## Extension Points

- Add new classification helpers to `IRSensor` when the logic is sensor-local.
- Keep multi-sensor orchestration in `IRSensorCalibration` or higher-level Python steps.
- If you change threshold semantics or persisted data shape, update the calibration-store docs and any UI-driven calibration flows at the same time.

## Testing

Relevant coverage lives in the Python binding tests and in higher-level motion/step workflows that consume the sensor. When changing the classification math, verify:

- black/white classification around the threshold edges
- behavior with noisy or poorly separated samples
- compatibility with persisted calibration data
