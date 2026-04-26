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

Steps are the user-facing building blocks of robot missions. They follow a two-layer pattern: a **hidden Step subclass** plus a **public factory** that users actually call. There are two ways to write that pair — pick one consistently within a module.

### Default: `@dsl_step` (codegen-generated factory)

Most new steps should use `@dsl_step` on the class. The `tools/generate_step_builders.py` codegen reads the class signature and emits the public factory + builder into a sibling `*_dsl.py` file. Less boilerplate, the docstring on the class becomes the factory docstring.

```python
from typing import TYPE_CHECKING
from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["category", "subcategory"])
class MyStep(Step):
    """One-line summary of what the step does.

    Paragraph explaining how it works internally — what controllers,
    sensors, or hardware it uses and how it achieves its goal.

    Prerequisites (if any):
        Mention required calibration, drivetrain type, sensors, etc.

    Args:
        param: Description of the parameter with units and default.

    Example::

        from raccoon.step.mymodule import my_step

        my_step(param=2.0)
    """

    def __init__(self, param: float = 1.0) -> None:
        super().__init__()
        self._param = param

    def _generate_signature(self) -> str:
        return f"MyStep(param={self._param:.2f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # Step logic here
        pass
```

The codegen produces `mymodule_dsl.py` next to the source. Generated files are committed and verified by a pre-commit hook (`python3 tools/generate_step_builders.py --check`) so drift is caught before CI.

### Manual: `@dsl(hidden=True)` + hand-written factory

Use the manual two-decorator pattern only when the factory needs logic the codegen can't express — e.g. converting user-friendly units (cm → m, deg → rad), wrapping multiple constructors behind one entry point, or branching on input type.

```python
from raccoon.step.annotation import dsl

@dsl(hidden=True)
class MyStep(Step):
    """Internal step — users go through my_step() below."""

    def __init__(self, distance_m: float) -> None:
        super().__init__()
        self._distance_m = distance_m

    # _generate_signature() and _execute_step() as above.


@dsl(tags=["category", "subcategory"])
def my_step(cm: float) -> MyStep:
    """User-facing factory — accepts centimeters, converts to meters."""
    return MyStep(distance_m=cm / 100.0)
```

### Key Rules

1. **Pick one decorator pattern per module** and stay consistent. New code should default to `@dsl_step`.
2. **Tags control documentation grouping** — always provide at least two tags (category + subcategory). The first tag is the primary category.
3. **Factory functions use user-friendly units** — centimeters (not meters), degrees (not radians). Convert internally.
4. **`_generate_signature()`** is required on every Step class — returns a string representation for logging/debugging.
5. **`_execute_step()`** for simple one-shot steps (async), or **`on_start()`/`on_update()`/`on_stop()`** lifecycle hooks for motion steps that subclass `MotionStep`.
6. **`TYPE_CHECKING` guard** — always put `from raccoon.robot.api import GenericRobot` behind `if TYPE_CHECKING:` and use string annotation `"GenericRobot"`.

### Motion Steps

Motion steps subclass `MotionStep` instead of `Step` and use a fixed-rate update loop:

```python
from .motion_step import MotionStep

@dsl_step(tags=["motion", "drive"])
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
