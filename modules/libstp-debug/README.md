# libstp-debug

## Purpose

`libstp-debug` is currently a very small companion module that adds a debug-break marker step to the step DSL.

## Public Python APIs

Implemented in `raccoon.step.debug_break`:

- `DebugBreakStep`
- `debug_break`

## Control Flow Model

- `DebugBreakStep` is a normal `Step`.
- With `--debug` (`LIBSTP_DEBUG=1`) it pauses the mission and waits for a hardware button press; without it the step only logs that the debug break was reached and returns immediately.

## Timing Database And Tracker Relationship

- Because it is a normal `Step`, debug-break execution is timed by `libstp-step` the same way as any other step when timing is enabled.
- `DebugBreakStep` does not override `_generate_signature()`, so all debug-break instances share the same timing signature regardless of label.

## Mission Lifecycle Relationship

- Debug breaks are only useful when inserted inside a mission step sequence.
- They can be placed in setup, main, or shutdown mission flows, but they do not alter mission scheduling on their own.

## Debugging Hooks

- The optional `label` is only used in the log message today.
- The step is decorated with `@dsl(hidden=True)`, so it stays available to code without becoming a normal user-facing DSL entry.

## Tests

- There are no tests inside `modules/libstp-debug`.
- I did not find separate automated coverage for the debug-break marker.

## Extension Points

- If this module grows beyond logging, keep the blocking/unblocking transport isolated in `DebugBreakStep._execute_step()`.
- If labels need separate timing baselines or analytics, override `_generate_signature()` to include them explicitly.
