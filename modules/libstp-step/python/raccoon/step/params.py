"""Typed, string-free mission parameters entered via the setup UI.

A *parameter* is a single value the operator dials in on the robot's screen
during the setup mission (e.g. a positional offset, a speed) and that the
normal mission code then reads back for its calculations.

The design goal is to **never expose a string key at the call site**.  A
parameter is declared once as a typed descriptor on a :class:`ParamSet`.
Python's ``__set_name__`` hands the descriptor its own attribute name, so the
internal storage key is derived automatically — a typo becomes an
``AttributeError`` at import time instead of a silently-wrong ``0.0`` at
runtime, and IDE autocomplete/rename work across the whole codebase.

Declare parameters once::

    from raccoon import NumberParam, ParamSet


    class P(ParamSet):
        cube_offset = NumberParam(default=0.0, unit="cm", min=-20, max=20, persist=True)
        ramp_speed = NumberParam(default=0.5, min=0.1, max=1.0)

Ask for them in the setup mission — ``.ask()`` returns a ready-to-use step and
pulls unit/range/default straight off the descriptor::

    class MySetup(SetupMission):
        def sequence(self) -> Sequential:
            return seq(
                [
                    P.cube_offset.ask("Cube-row offset"),
                    P.ramp_speed.ask("Ramp speed"),
                ]
            )

Read them in normal mission code — typed, autocompleted, rename-safe::

    def sequence(self) -> Sequential:
        off = P.cube_offset.get()  # -> float
        return seq([drive_forward(cm=60 + off)])

Or bind the value lazily inside a declarative ``seq([...])`` tree with the
existing :func:`defer` primitive, so the calculation happens at execution
time regardless of build order::

    seq(
        [
            defer(lambda r: drive_forward(cm=60 + P.cube_offset.get())),
        ]
    )

Persistence is opt-in: ``persist=True`` mirrors the value into
``racoon.calibration.yml`` (via the shared :class:`CalibrationStore`) so it
survives a process restart and is reused under ``--no-calibrate``.  Without it
a parameter lives only in RAM for the current process — set once in setup,
read by every later mission.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from raccoon.no_calibrate import is_no_calibrate
from raccoon.ui.events import on_change, on_click, on_keypad
from raccoon.ui.screen import UIScreen
from raccoon.ui.step import UIStep
from raccoon.ui.widgets import (
    Button,
    NumericInput,
    NumericKeypad,
    Row,
    Spacer,
    Split,
    Text,
    Widget,
)

from .annotation import dsl
from .base import Step
from .calibration.store import CalibrationStore

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Storage
# ---------------------------------------------------------------------------

#: YAML section (inside ``racoon.calibration.yml``) that persisted parameters
#: live under.  Each parameter is its own ``set_name`` there, so parameters
#: never clobber each other or the sensor-calibration sections.
_PARAM_SECTION = "params"
_VALUE_KEY = "value"


class _ParamStore:
    """Runtime value cache with optional YAML persistence.

    Runtime values live in ``self._values`` for the lifetime of the process
    and are shared across every mission.  When a parameter opts into
    persistence, the value is additionally written to (and lazily read back
    from) the shared calibration YAML so it survives restarts.
    """

    def __init__(self, cal_store: CalibrationStore | None = None) -> None:
        self._values: dict[str, Any] = {}
        self._cal = cal_store or CalibrationStore()

    def set(self, key: str, value: Any, *, persist: bool = False) -> None:
        self._values[key] = value
        if persist:
            self._cal.store(_PARAM_SECTION, {_VALUE_KEY: value}, set_name=key)

    def get(self, key: str, default: Any, *, persisted: bool = False) -> Any:
        if key in self._values:
            return self._values[key]
        if persisted:
            entry = self._cal.load(_PARAM_SECTION, set_name=key)
            if entry is not None and _VALUE_KEY in entry:
                value = entry[_VALUE_KEY]
                self._values[key] = value  # cache so later reads skip disk
                return value
        return default

    def has(self, key: str, *, persisted: bool = False) -> bool:
        if key in self._values:
            return True
        if persisted:
            entry = self._cal.load(_PARAM_SECTION, set_name=key)
            return entry is not None and _VALUE_KEY in entry
        return False

    def clear(self) -> None:
        """Drop all runtime values (persisted YAML is untouched)."""
        self._values.clear()


#: Process-wide singleton.  Parameters read/write through this.
_store = _ParamStore()


def reset_params() -> None:
    """Clear all runtime parameter values (test helper).

    Only touches the in-memory cache; persisted YAML entries are left alone.
    """
    _store.clear()


# ---------------------------------------------------------------------------
# Parameter descriptors
# ---------------------------------------------------------------------------


class NumberParam:
    """A numeric mission parameter, declared once as a :class:`ParamSet` field.

    The descriptor owns the value's metadata (default, unit, valid range,
    persistence) and, via ``__set_name__``, its own storage key — so callers
    interact with the *attribute*, never a string::

        class P(ParamSet):
            cube_offset = NumberParam(default=0.0, unit="cm", min=-20, max=20)


        P.cube_offset.get()  # -> float
        P.cube_offset.ask("Offset...")  # -> Step for the setup mission

    Args:
        default: Value returned by :meth:`get` before anything is entered.
        unit: Display unit shown on the input screen (e.g. ``"cm"``).
        min: Lower clamp bound, or ``None`` for unbounded.
        max: Upper clamp bound, or ``None`` for unbounded.
        persist: When ``True``, entered values are mirrored to
            ``racoon.calibration.yml`` and reused across restarts /
            ``--no-calibrate``.
        key: Explicit storage key.  Normally omitted — the attribute name is
            used automatically.  Only needed when declaring a parameter
            outside a class body (e.g. a module-level constant).
    """

    def __init__(
        self,
        default: float = 0.0,
        *,
        unit: str = "",
        min: float | None = None,
        max: float | None = None,
        persist: bool = False,
        key: str | None = None,
    ) -> None:
        self._key = key
        self.default = float(default)
        self.unit = unit
        self.min = None if min is None else float(min)
        self.max = None if max is None else float(max)
        self.persist = persist

    def __set_name__(self, owner: type, name: str) -> None:
        if self._key is None:
            self._key = name

    @property
    def key(self) -> str:
        if self._key is None:
            msg = (
                "NumberParam has no key — declare it as a class attribute on a "
                "ParamSet, or pass key=... for a standalone parameter."
            )
            raise RuntimeError(msg)
        return self._key

    def _clamp(self, value: float) -> float:
        if self.min is not None:
            value = max(self.min, value)
        if self.max is not None:
            value = min(self.max, value)
        return value

    def get(self) -> float:
        """Return the current value, or :attr:`default` if none was entered."""
        return float(_store.get(self.key, self.default, persisted=self.persist))

    def set(self, value: float) -> None:
        """Set the value directly (clamped to the declared range)."""
        _store.set(self.key, self._clamp(float(value)), persist=self.persist)

    def is_set(self) -> bool:
        """Whether a value has been entered/persisted (vs. falling back to default)."""
        return _store.has(self.key, persisted=self.persist)

    def ask(self, prompt: str, *, title: str = "Setup") -> "Step":
        """Return a setup step that asks the operator for this value.

        The input screen is pre-filled with the current value (persisted or
        default) and constrained to the declared unit and range.  Cancelling
        keeps the existing value.  Under ``--no-calibrate`` the UI is skipped
        and the persisted/default value is used as-is.

        Args:
            prompt: Question shown above the numeric keypad.
            title: Screen title. Defaults to ``"Setup"``.

        Returns:
            An :class:`AskNumber` step, ready to drop into a ``seq([...])``.
        """
        return AskNumber(self, prompt, title=title)


class ParamSet:
    """Base for a group of parameter declarations.

    Subclassing is optional — ``__set_name__`` works on any class — but it
    provides :meth:`all` for iterating declared parameters (handy for tests
    or a "reset everything" screen).

    Example::

        class P(ParamSet):
            cube_offset = NumberParam(default=0.0, unit="cm")
            ramp_speed = NumberParam(default=0.5)
    """

    @classmethod
    def all(cls) -> list[NumberParam]:
        """Return every :class:`NumberParam` declared on this class."""
        return [v for v in vars(cls).values() if isinstance(v, NumberParam)]


# ---------------------------------------------------------------------------
# Setup step
# ---------------------------------------------------------------------------


class _AskNumberScreen(UIScreen[float | None]):
    """Numeric keypad screen, mirroring the distance-calibration layout.

    Uses the same ``Split`` composition as ``DistanceMeasureScreen`` — a
    ``NumericKeypad`` on the right, and on the left the prompt, a live
    ``NumericInput`` (which echoes the typed value and offers +/- adjust),
    and the action buttons — so it fits the screen the same way.  Returns the
    entered ``float`` clamped to the declared range, or ``None`` if cancelled.
    The physical robot button confirms (= Submit).
    """

    _primary_button_id = "submit"

    def __init__(
        self,
        prompt: str,
        *,
        initial_value: float = 0.0,
        unit: str = "",
        min_value: float | None = None,
        max_value: float | None = None,
        title: str = "Setup",
    ) -> None:
        super().__init__()
        self.title = title
        self._prompt = prompt
        self._unit = unit
        self._min = min_value
        self._max = max_value
        self.value = self._clamp(initial_value)
        self._input_str = ""  # raw keypad input; empty means "showing default"

    def _clamp(self, value: float) -> float:
        if self._min is not None:
            value = max(self._min, value)
        if self._max is not None:
            value = min(self._max, value)
        return value

    def _range_hint(self) -> str:
        lo, hi, unit = self._min, self._max, self._unit
        if lo is not None and hi is not None:
            return f"Range: {lo:g} – {hi:g} {unit}".rstrip()
        if lo is not None:
            return f"Min: {lo:g} {unit}".rstrip()
        if hi is not None:
            return f"Max: {hi:g} {unit}".rstrip()
        return ""

    def build(self) -> Widget:
        # Keep the prompt at "medium" (not "title"): a long prompt rendered at
        # title size stretches the column and pushes the value + Submit button
        # off the bottom of the screen.
        left: list[Widget] = [
            Text(self._prompt, size="medium", bold=True),
        ]
        hint = self._range_hint()
        if hint:
            left += [Spacer(4), Text(hint, size="small", muted=True)]
        left += [
            Spacer(16),
            NumericInput(
                id="value",
                value=self.value,
                unit=self._unit,
                min_value=self._min,
                max_value=self._max,
            ),
            Spacer(24),
            Row(
                children=[
                    Button("cancel", "Cancel", style="secondary"),
                    Button("submit", "Submit", style="success", icon="check"),
                ],
                spacing=12,
            ),
        ]
        return Split(
            left=left,
            right=[
                NumericKeypad(),
            ],
            ratio=(5, 6),
        )

    @on_keypad()
    async def on_key(self, key: str):
        if key == "back":
            self._input_str = self._input_str[:-1]
        elif key == "." and "." not in self._input_str:
            self._input_str += "."
        elif key.isdigit():
            self._input_str += key

        try:
            self.value = self._clamp(float(self._input_str)) if self._input_str else 0.0
        except ValueError:
            self.value = 0.0

        await self.refresh()

    @on_change("value")
    async def on_adjust(self, value: float):
        # +/- buttons on the NumericInput
        self.value = self._clamp(value)
        self._input_str = f"{self.value:g}"
        await self.refresh()

    @on_click("submit")
    async def on_submit(self):
        self.close(self.value)

    @on_click("cancel")
    async def on_cancel(self):
        self.close(None)


@dsl(hidden=True)
class AskNumber(UIStep):
    """Setup step: ask the operator for a numeric parameter via the screen.

    Prefer ``param.ask("...")`` or the :func:`ask` factory over constructing
    this directly.  Shows a numeric keypad pre-filled with the parameter's
    current value, clamped to its declared range, and stores whatever the
    operator confirms back into the parameter.

    Under ``--no-calibrate`` the screen is skipped entirely and the existing
    (persisted or default) value is kept, matching the behaviour of the
    calibration steps.
    """

    def __init__(self, param: NumberParam, prompt: str, *, title: str = "Setup") -> None:
        super().__init__()
        self._param = param
        self._prompt = prompt
        self._title = title

    def _generate_signature(self) -> str:
        return f"AskNumber(key={self._param.key!r}, value={self._param.get():.2f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if is_no_calibrate():
            robot.info(
                f"--no-calibrate: {self._param.key} = "
                f"{self._param.get():g}{self._param.unit} (input skipped)"
            )
            return

        screen = _AskNumberScreen(
            self._prompt,
            initial_value=self._param.get(),
            unit=self._param.unit,
            min_value=self._param.min,
            max_value=self._param.max,
            title=self._title,
        )
        value = await self.show(screen)

        if value is None:
            robot.warn(
                f"{self._param.key}: input cancelled, keeping "
                f"{self._param.get():g}{self._param.unit}"
            )
            return

        self._param.set(value)
        robot.info(
            f"[param] {self._param.key} = {self._param.get():g}{self._param.unit} "
            f"(entered via setup UI)"
        )


@dsl(tags=["control", "params"])
def ask(param: NumberParam, prompt: str, *, title: str = "Setup") -> "Step":
    """Ask the operator for a numeric parameter during setup.

    Free-function form of :meth:`NumberParam.ask`; both build the same step.
    Some prefer ``ask(P.cube_offset, "...")`` over ``P.cube_offset.ask("...")``.

    Args:
        param: The :class:`NumberParam` to fill in.
        prompt: Question shown above the numeric keypad.
        title: Screen title. Defaults to ``"Setup"``.

    Returns:
        An :class:`AskNumber` step, ready to drop into a ``seq([...])``.

    Example::

        from raccoon import ask, NumberParam, ParamSet


        class P(ParamSet):
            cube_offset = NumberParam(default=0.0, unit="cm")


        seq([ask(P.cube_offset, "Cube-row offset")])
    """
    return param.ask(prompt, title=title)
