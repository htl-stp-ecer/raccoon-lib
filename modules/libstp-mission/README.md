# libstp-mission

## Purpose

`libstp-mission` defines the thin mission abstraction used by the robot runner. A mission is primarily a named unit that builds a root step sequence and delegates actual execution to `libstp-step`.

This module is intentionally small. It does not currently implement a full controller layer.

## Public Python APIs

Implemented in `libstp.mission.api`:

- `MissionProtocol`
- `Mission`
- `MissionController`

## Control Flow Model

- `Mission.run(robot)` logs start/end and executes `self.sequence().run_step(robot)`.
- `Mission.sequence()` is abstract and must return a `Step`.
- `MissionController.execute_missions()` is present as a placeholder but currently raises `NotImplementedError`.

## Timing Database And Tracker Relationship

- Missions do not talk to the timing database directly.
- Timing comes from the steps returned by `sequence()`, because `Step.run_step()` is where `libstp-timing` instrumentation lives.
- If a mission constructs different step graphs over time, that will change the step signatures collected by the timing tracker.

## Mission Lifecycle

The actual lifecycle lives in `libstp-robot`, not in this module. As of the current code, the runner does the following:

1. Preload setup, main, and shutdown missions by calling `sequence()` early.
2. Reset the timer and run the optional setup mission.
3. Run the pre-start gate, which waits for either a button press or a wait-for-light calibration flow.
4. Reset the timer again, start the synchronizer recording, and execute main missions sequentially under the shutdown timeout.
5. Reset the timer again and run the optional shutdown mission.

Contributor implication:

- Keep `sequence()` side-effect free. The robot runner calls it during preload before mission execution starts.

## Debugging Hooks

- `Mission` inherits `ClassNameLogger`, so mission boundaries are logged.
- Step-level debugging is done inside the returned sequence, including breakpoint markers from `libstp-debug`.

## Tests

- There are no tests inside `modules/libstp-mission`.
- I did not find direct automated coverage for the mission abstractions themselves.
- Any real validation currently depends on robot-runner integration.

## Extension Points

- Subclass `Mission` and return a composed step tree from `sequence()`.
- Put timing-sensitive or hardware side effects inside steps, not inside `sequence()`.
- If a real mission controller is needed, either finish `MissionController` or move lifecycle ownership into `libstp-robot` where the actual orchestration already lives.
