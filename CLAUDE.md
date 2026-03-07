# Botball Library

## Build

```bash
RPI_HOST=10.101.156.206 bash deploy.sh
```

## Project Structure

- `modules/libstp-step/` — Step base class, annotation system, control flow (seq, parallel, loop, defer), motor, servo, wait, calibration steps
- `modules/libstp-motion/` — Motion steps (drive, turn, strafe, lineup, move_until, line_follow, wall_align, tuning)
- `modules/libstp-servo/` — Servo steps
- `modules/libstp-timing/` — Timing/checkpoint steps
- `modules/libstp-hal/` — Hardware abstraction layer (C++ with pybind11 bindings)
- `modules/libstp-platforms/` — Platform implementations (wombat, mock)
- `python/stubs/` — `.pyi` stubs for C++ bindings
- `docs/` — Sphinx docs config, DSL catalog generator
- `tests/` — C++ and Python tests

## DSL Step Conventions

Steps are the user-facing building blocks of robot missions. They follow a strict two-layer pattern:

### Architecture: Hidden Class + Public Factory Function

Every step is a **hidden Step subclass** paired with one or more **public factory functions**:

```python
from typing import TYPE_CHECKING
from .. import Step, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class MyStep(Step):
    """Brief description of the step implementation."""

    def __init__(self, param: float) -> None:
        super().__init__()
        self._param = param

    def _generate_signature(self) -> str:
        return f"MyStep(param={self._param:.2f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # Step logic here
        pass


@dsl(tags=["category", "subcategory"])
def my_step(param: float = 1.0) -> MyStep:
    """One-line summary of what the step does.

    Paragraph explaining how it works internally — what controllers,
    sensors, or hardware it uses and how it achieves its goal.

    Prerequisites (if any):
        Mention required calibration, drivetrain type, sensors, etc.

    Args:
        param: Description of the parameter with units and default.

    Returns:
        A MyStep instance configured for the described behavior.

    Example::

        from libstp.step.xxx import my_step

        # Brief comment explaining the example
        my_step(param=2.0)
    """
    return MyStep(param)
```

### Key Rules

1. **Step classes are always `@dsl(hidden=True)`** — users never instantiate them directly.
2. **Factory functions are `@dsl(tags=[...])`** — these are the public API. Tags control grouping in the docs catalog. Always provide at least two tags (category + subcategory).
3. **Factory functions use user-friendly units** — centimeters (not meters), degrees (not radians). Convert internally.
4. **`_generate_signature()`** is required on every Step class — returns a string representation for logging/debugging.
5. **`_execute_step()`** for simple one-shot steps (async), or **`on_start()`/`on_update()`/`on_stop()`** lifecycle hooks for motion steps that subclass `MotionStep`.
6. **`TYPE_CHECKING` guard** — always put `from libstp.robot.api import GenericRobot` behind `if TYPE_CHECKING:` and use string annotation `"GenericRobot"`.

### Motion Steps

Motion steps subclass `MotionStep` instead of `Step` and use a fixed-rate update loop:

```python
from .motion_step import MotionStep

@dsl(hidden=True)
class MyMotion(MotionStep):
    def on_start(self, robot: "GenericRobot") -> None:
        # Initialize motion controller, set velocity, etc.
        pass

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        # Update controller, return True when motion is complete
        return self._motion.is_finished()

    def on_stop(self, robot: "GenericRobot") -> None:
        # Optional: default calls robot.drive.hard_stop()
        pass
```

### Docstring Requirements for Factory Functions

Every public `@dsl` factory function MUST have a complete docstring with:

1. **One-line summary** — what the step does (imperative mood: "Drive forward", not "Drives forward")
2. **How it works** — paragraph explaining the mechanism (PID control, sensor polling, velocity profile, etc.)
3. **Prerequisites** — if the step requires calibration, specific drivetrain, sensors, or prior steps
4. **Args** — all parameters with types, units, defaults, and valid ranges
5. **Returns** — what Step type is returned
6. **Raises** — if it can raise (e.g., `CalibrationRequiredError`)
7. **Example::** — realistic Python code showing common usage (use `::` after Example for RST literal block)

### Tag Categories

Use these established tag groups (first tag = primary category):

| Primary Tag | Use For |
|---|---|
| `motion` | Drive, turn, strafe, wall-align, stop |
| `motor` | Individual motor control |
| `servo` | Servo control |
| `sensor` | Sensor-triggered steps (move_until) |
| `calibration` | Calibration and tuning |
| `control` | Control flow (loop, defer, do_while, timeout) |
| `timing` | Checkpoint-based timing |

Common secondary tags: `drive`, `turn`, `strafe`, `wall`, `stop`, `lineup`, `line-follow`, `wait`, `loop`, `actuator`, `auto-tune`, `sync`

## Documentation

- `docs/generate_dsl_catalog.py` — AST-based scanner that extracts all `@dsl` functions into `dsl-steps.json`
- The Hugo documentation site at `htl-stp-ecer/documentation` consumes this JSON
- Sphinx autoapi generates Python API docs; Doxygen generates C++ API docs
- Run `python3 docs/generate_dsl_catalog.py` to regenerate the catalog after changing step docstrings
