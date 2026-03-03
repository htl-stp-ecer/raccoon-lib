# libstp-timing

## Purpose

`libstp-timing` records step runtimes, keeps a rolling baseline in SQLite, reports anomalies, and provides a mission-relative checkpoint synchronizer.

The module has two layers:

- `libstp.timing.*` for runtime timing collection and anomaly detection
- `libstp.step.timing.*` for step wrappers that wait for or run until a synchronizer checkpoint

## Public Python APIs

Exports from `libstp.timing`:

- `StepTimingTracker`
- `AnomalyCallback`
- `AnomalyDetection`
- `StepStatistics`
- `TimingConfig`
- `Synchronizer`

Contributor-relevant public classes not exported from `libstp.timing.__all__`:

- `StepTimingDatabase`

Step helpers implemented in this module under `libstp.step.timing`:

- `DoUntilCheckpoint`
- `do_until_checkpoint`
- `WaitForCheckpoint`
- `wait_for_checkpoint`

## Control Flow Model

- `Step.run_step()` in `libstp-step` calls `StepTimingTracker.get_instance()` and records each completed execution when timing is enabled.
- `StepTimingTracker.record_execution()` serializes access with an async lock, loads recent non-anomalous samples for the step signature, computes statistics, checks for an anomaly, fires callbacks, then stores the new sample.
- `Synchronizer.start_recording()` captures mission-relative time zero from the current event loop clock.
- `wait_for_checkpoint()` blocks until a checkpoint is reached.
- `do_until_checkpoint()` runs one child step until the checkpoint deadline, then cancels it.

## Timing Database And Tracker Concepts

SQLite schema:

- One `step_executions` table keyed by auto-increment id.
- Each row stores `step_signature`, `duration_seconds`, `timestamp_unix`, and anomaly metadata.
- Recent baseline samples are fetched in reverse timestamp order and exclude rows already flagged as anomalies.

Anomaly model:

- Baseline requires at least two prior non-anomalous samples.
- Mean and sample standard deviation are computed from the configured rolling window.
- A run is anomalous when `duration` falls outside `mean +/- threshold_multiplier * stddev`.
- Callbacks receive an `AnomalyDetection` record before the new sample is inserted.

## Mission Lifecycle Relationship

This module does not start or stop missions. It relies on other layers to give it stable timing boundaries:

- `libstp-step` decides when individual step timings are recorded.
- `libstp-robot` resets the mission timer and calls `robot.synchronizer.start_recording()` immediately before main missions begin.
- Checkpoint steps only make sense after `start_recording()` has been called. Using them in setup code before that point will operate against an unset start time.

## Debugging Hooks

- `StepTimingTracker.register_anomaly_callback()` lets you surface anomalies to logs, UI, telemetry, or breakpoints.
- The tracker logs cold-start baseline establishment for the first two samples of a signature.
- The synchronizer logs when a checkpoint was missed, reached, or waited on.

## Tests

- There are no tests inside `modules/libstp-timing`.
- I did not find direct automated coverage for the tracker, database, or synchronizer code.
- In practice this code is exercised indirectly when step execution timing is enabled during robot runs.

## Extension Points

- Provide a custom `TimingConfig` to change the anomaly threshold, window size, database path, or to disable timing entirely.
- Override step `_generate_signature()` implementations in `libstp-step`; signature quality is what makes this module useful.
- Add anomaly callbacks instead of baking policy into the tracker.
- Extend `libstp.step.timing` with new checkpoint-aware steps if you need mission-relative coordination without changing the tracker/database layer.
