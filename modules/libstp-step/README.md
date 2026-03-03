# libstp-step

## Purpose

`libstp-step` defines the async step abstraction used to build robot behavior. It owns the base `Step` contract, composition helpers such as sequential and parallel execution, simple wait/control helpers, and the calibration flows that are implemented as UI-backed steps.

This module does not own the robot lifecycle. Missions and the robot runner decide when steps are constructed and executed.

## Public Python APIs

Package exports from `libstp.step` implemented in this module:

- `Step`
- `StepProtocol`
- `SimulationStep`
- `SimulationStepDelta`
- `DslMeta`
- `dsl`
- `Sequential`
- `seq`
- `parallel`
- `WaitForSeconds`
- `wait`
- `WaitForButton`
- `wait_for_button`

Logic helpers re-exported from `libstp.step.logic`:

- `DoWhileActive`
- `do_while_active`
- `LoopForeverStep`
- `loop_forever`
- `LoopForStep`
- `loop_for`

Calibration exports re-exported from `libstp.step.calibration`:

- `calibrate`
- `calibrate_wait_for_light`
- `CalibrateWaitForLight`
- `calibrate_distance`
- `CalibrateDistance`
- `CalibrationRequiredError`
- `PerWheelCalibration`
- `is_distance_calibrated`
- `check_distance_calibration`
- `reset_distance_calibration`
- `calibrate_deadzone`
- `CalibrateDeadzone`
- `DeadzoneCalibrationResult`

Public classes also live below the package surface but are not re-exported through `libstp.step.__all__`, notably the IR sensor calibration flow in `libstp.step.calibration.sensors`.

## Control Flow Model

- `Step.run_step(robot)` is the outer execution wrapper. It logs start/finish, calls `_execute_step(robot)`, and records timing when `libstp-timing` is enabled.
- `Sequential` runs child steps in list order.
- `parallel()` normalizes each branch into a step, wrapping bare steps and step lists in `Sequential`, then waits for every branch to finish.
- `Timeout` uses `asyncio.wait_for(...)` around one child step. On timeout it logs an error and completes without re-raising `asyncio.TimeoutError`.
- `DoWhileActive` starts a reference step and a task step together, then cancels the task step when the reference step finishes.
- `loop_forever()` and `loop_for()` rerun the same child step instance. Any mutable state held on that child persists across iterations.

## Timing Database And Tracker Relationship

Timing persistence lives in `libstp-timing`, but every `Step` participates through two hooks:

- `_generate_signature()` decides which executions share timing history. Override it whenever constructor parameters materially change runtime.
- `to_simulation_step()` uses timing history only when it can synchronously query the timing database. If it is called while an event loop is already running, the base implementation falls back to default duration values unless a subclass overrides it.

## Calibration Notes

- `calibrate()` is a convenience wrapper around `CalibrateDistance(..., calibrate_light_sensors=True)`.
- `CalibrateDistance` updates drive motor calibration in memory immediately after confirmation and can persist EMA-smoothed `ticks_to_rad` values back to `raccoon.project.yml`.
- `check_distance_calibration()` currently only logs a warning. It does not raise `CalibrationRequiredError`, even though the exception type exists.
- `CalibrateDeadzone` is operator-driven. It updates each motor definition's `ff.kS` in memory after the summary screen is confirmed.
- The IR calibration flows are UI-heavy and depend on `UIStep` behavior from the screen module. They are best validated on-device.

## Mission Lifecycle Relationship

Steps are normally produced by `Mission.sequence()` implementations and executed by the robot runner in `libstp-robot`.

Important contributor constraint:

- `libstp-robot` preloads missions by calling `sequence()` before execution starts.

That means step construction should stay cheap and side-effect free. Hardware access belongs in `_execute_step()`, not in mission builders or step constructors that run during preload.

## Debugging Hooks

- Every step inherits `ClassNameLogger`, so debug/info/warn/error logging is available inside `_execute_step()`.
- `@dsl(...)` attaches discovery metadata used by higher-level tooling. `hidden=True` keeps helper/composite steps out of discovery.
- The sibling `libstp-debug` module provides `breakpoint()` as a step-level marker that can be inserted into any sequence.

## Tests

- There are no tests inside `modules/libstp-step`.
- The only directly related automated coverage I found is top-level `tests/python/test_calibrate_distance.py`.
- That test file is only partial and currently stale in places: it expects `check_distance_calibration()` to raise and references `_parse_measured_distance()`, which does not exist in the current implementation.
- UI calibration flows and control-step interactions still need manual or higher-level integration testing.

## Extension Points

- Subclass `Step` and implement `_execute_step(robot)`.
- Override `_generate_signature()` for parameterized steps so timing history stays useful.
- Override `to_simulation_step()` if the step changes robot pose or has deterministic duration data.
- Wrap public factories and classes in `@dsl(...)` so they carry UI/discovery metadata.
- Follow the existing UI calibration steps when handling screen dismissal or navigation pop events; those flows explicitly abort to avoid reopening stale screens.
