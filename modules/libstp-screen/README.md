# libstp-screen

`libstp-screen` contains the dynamic UI framework used by calibration and operator-facing workflows.

It is a Python-first module. Screens are declared as Python classes that build widget trees, and those widget trees are serialized onto an LCM transport channel for an external renderer to display.

## Public API

Python package:

- `python/libstp/ui/`

Core classes:

- `UIScreen`
- `UIStep`

Supporting layers:

- `widgets.py` for the serializable widget tree
- `events.py` for event-binding decorators
- `screens/` for reusable screen implementations
- `raccoon/` for the generated LCM message classes used on the wire

## Architecture

The split of responsibility is important:

- `UIScreen` owns local UI state, widget layout, and event handlers.
- `UIStep` owns orchestration: showing screens, pumping events, and tying the UI to the step lifecycle.
- The renderer is intentionally outside this module; the contract is the serialized widget tree and the screen render / answer messages.

## Transport Contract

This module currently publishes dynamic UI payloads on:

- `libstp/screen_render`
- `libstp/screen_render/answer`

`UIScreen._to_dict()` is the stable internal format for widget serialization in this module. If contributors change that structure, they must update the corresponding renderer at the same time.

## Extension Points

- Add reusable widgets when the shape is renderer-agnostic and broadly useful.
- Add new prebuilt screens under `screens/` when a pattern is used by multiple workflows.
- Keep ad hoc workflow state in custom `UIScreen` subclasses rather than in `UIStep`.
- Preserve the event-decorator pattern so screens remain easy to inspect and test.

## Testing And Validation

Most confidence for this module comes from higher-level step and calibration flows rather than isolated unit tests. When editing the screen contract, verify:

- widget serialization
- event dispatch
- UI cleanup on step failure or cancellation
- compatibility with the renderer consuming `screen_render_t` and `screen_render_answer_t`
