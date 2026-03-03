# libstp-debug

## Purpose

`libstp-debug` is currently a very small companion module that adds a breakpoint marker step to the step DSL.

## Public Python APIs

Implemented in `libstp.step.breakpoint`:

- `BreakpointStep`
- `breakpoint`

## Control Flow Model

- `BreakpointStep` is a normal `Step`.
- At runtime it only logs that the breakpoint was reached and then returns immediately.
- There is no interactive pause or remote unblock mechanism yet. The file still contains a TODO for that behavior.

## Timing Database And Tracker Relationship

- Because it is a normal `Step`, breakpoint execution is timed by `libstp-step` the same way as any other step when timing is enabled.
- `BreakpointStep` does not override `_generate_signature()`, so all breakpoint instances share the same timing signature regardless of label.

## Mission Lifecycle Relationship

- Breakpoints are only useful when inserted inside a mission step sequence.
- They can be placed in setup, main, or shutdown mission flows, but they do not alter mission scheduling on their own.

## Debugging Hooks

- The optional `label` is only used in the log message today.
- The step is decorated with `@dsl(hidden=True)`, so it stays available to code without becoming a normal user-facing DSL entry.

## Tests

- There are no tests inside `modules/libstp-debug`.
- I did not find separate automated coverage for the breakpoint marker.

## Extension Points

- If this module grows beyond logging, keep the blocking/unblocking transport isolated in `BreakpointStep._execute_step()`.
- If labels need separate timing baselines or analytics, override `_generate_signature()` to include them explicitly.
