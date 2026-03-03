# libstp-calibration

`libstp-calibration` contains the low-level routines used to measure and tune motor and motion-control parameters. It is the calibration engine that other parts of the stack build on, not a general end-user workflow module by itself.

There are two distinct concerns in this module:

- Motor calibration: identify feedforward constants and PID gains for a single motor.
- Heading autotune while driving straight: estimate turn-loop gains from a relay test.

The current code does **not** expose a generic `MotionCalibrator` class, and the Python bindings only export result/config datatypes for motor calibration. Contributors should treat the C++ headers as the source of truth.

## Responsibilities

- Measure static friction, velocity constant, and acceleration constant.
- Tune velocity PID gains with either step-response or relay-feedback methods.
- Validate calibrated values against configurable safety and range checks.
- Export calibration metrics for debugging and review.
- Provide a separate relay-feedback autotuner for straight-driving heading control.

## Public C++ API

Primary headers:

- `include/calibration/motor/calibration.hpp`
- `include/calibration/motor/calibration_config.hpp`
- `include/calibration/motor/calibration_result.hpp`
- `include/calibration/motion/drive_straight_autotune.hpp`

Supporting public headers under `include/calibration/motor/` expose the internal building blocks used by `MotorCalibrator`:

- `control/` for hardware access wrappers
- `data/` for sampled velocity data
- `feedforward/` for `kS`, `kV`, and `kA` estimation
- `pid/` for tuning strategies
- `analysis/` for signal extraction
- `validation/` for guardrails and post-checks
- `utils/` for shared math/time helpers

## Motor Calibration Flow

`MotorCalibrator` is the main orchestration class.

Typical sequence:

1. Construct a `MotorCalibrator` with an `IMotor` and `CalibrationConfig`.
2. Call `calibrate()`.
3. Inspect the returned `CalibrationResult`.
4. Persist or apply the tuned `Feedforward` and `PidGains` where your platform expects them.

You can also run the stages separately:

- `calibrateFeedforward()`
- `calibratePID(ff)`

## Heading Autotune Flow

`DriveStraightAutotuner` is narrower in scope than the old README implied:

- It only autotunes the heading loop used while driving straight.
- It runs a relay-feedback experiment against `Drive` and `IOdometry`.
- It returns a `DriveStraightAutotuneResult` containing the measured oscillation data and derived PID gains.

## Python Bindings

`libstp.calibration` currently exposes:

- `CalibrationConfig`
- `CalibrationResult`
- `CalibrationMetrics`

It does **not** currently expose:

- `MotorCalibrator`
- `DriveStraightAutotuner`
- the helper component classes under `motor/`

If you need those from Python, you must extend the bindings first.

## Safety And Operational Assumptions

This module can command real motors aggressively. Contributors should assume:

- The motor under test is free to move through the configured travel distance.
- Encoder feedback is working and scaled correctly.
- The test environment can tolerate repeated starts, stops, and oscillation experiments.
- Safety limits in `CalibrationConfig` are last-resort guardrails, not a replacement for supervision.

Relevant safety knobs include:

- `max_test_distance_m`
- `max_single_test_duration`
- `max_calibration_duration`
- `max_retries`
- `validate_parameter_ranges`

## Extension Points

To add a new motor-calibration stage:

1. Put the implementation in the appropriate `motor/` subdirectory.
2. Keep it focused on one concern.
3. Thread it into `MotorCalibrator` only after its inputs/outputs are clear.
4. Add result fields only when the data is useful outside the helper itself.

To add a new motion autotuner:

1. Place it under `include/calibration/motion/` and `src/motion/`.
2. Keep it separate from `MotorCalibrator`.
3. Document exactly which loop it tunes and what runtime assumptions it makes.

## Testing

This module is validated indirectly by:

- the calibration-related binding tests
- drive and motion tests that consume tuned constants
- runtime workflows in `libstp.step.motion` and `libstp.step.calibration`

When changing calibration math, verify both the numeric outputs and the failure modes. New code in this module should be explicit about what happens when the signal is too noisy, the motor saturates, or the robot cannot complete the requested test safely.
