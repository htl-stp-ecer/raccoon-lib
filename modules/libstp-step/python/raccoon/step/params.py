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
        cube_offset = NumberParam(default=0.0, unit="cm", persist=True)
        ramp_speed = NumberParam(default=0.5)

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

Profiles (game-table / side scoping)
-------------------------------------

Some parameters differ per physical game table and per starting side.  Mark
such a parameter ``scoped=True`` and it is stored *under the active profile*
— a ``(game-table, side)`` pair chosen once during setup — instead of
globally.  ``.get()`` transparently resolves against whatever profile is
active, so mission code never mentions the table or side::

    class P(ParamSet):
        cube_offset = NumberParam(default=0.0, unit="cm", scoped=True, persist=True)
        drum_speed = NumberParam(default=0.5)  # global — unaffected by profile

Drive the whole flow from the setup mission with a single step: it shows one
keypad screen where the operator types the table number and taps ``A`` / ``B``
(e.g. ``1B``) and — if a *complete* set of saved values already exists for that
profile — offers to reuse them (asking first) instead of re-entering every
value::

    class MySetup(SetupMission):
        def sequence(self) -> Sequential:
            return seq(
                [
                    configure_mission(
                        params=[
                            (P.cube_offset, "Cube-row offset"),
                        ],
                    ),
                ]
            )

Un-scoped parameters ignore the active profile completely, so mixing scoped
and global parameters in one mission is fine.
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

    Only touches the in-memory cache and the active profile; persisted YAML
    entries are left alone.
    """
    _store.clear()
    clear_active_profile()


# ---------------------------------------------------------------------------
# Active profile (game-table / side)
# ---------------------------------------------------------------------------

#: YAML ``set_name`` (inside the ``params`` section) the selected profile is
#: mirrored under so ``--no-calibrate`` / a restart can restore it silently.
_ACTIVE_PROFILE_KEY = "_active_profile"


class _ActiveProfile:
    """Mutable holder for the process-wide selected profile.

    A tiny box (mirroring :data:`_store`) so the module-level functions mutate
    an attribute instead of rebinding a module global — the value is the active
    ``(game-table, side)`` pair, or ``None`` before a profile is chosen.  A
    scoped :class:`NumberParam` reads/writes under this pair; an un-scoped one
    ignores it.  Set once during setup, read by every later mission.
    """

    def __init__(self) -> None:
        self.value: tuple[str, str] | None = None


_profile = _ActiveProfile()


def set_active_profile(table: str, side: str, *, persist: bool = True) -> None:
    """Select the active ``(game-table, side)`` profile for scoped parameters.

    Args:
        table: Game-table name (as shown in the setup menu).
        side: Starting side (e.g. ``"A"`` / ``"B"``).
        persist: When ``True`` (default) the selection is mirrored into the
            calibration YAML so a restart / ``--no-calibrate`` run can restore
            it without re-asking.
    """
    _profile.value = (table, side)
    if persist:
        _store.set(
            _ACTIVE_PROFILE_KEY,
            {"table": table, "side": side},
            persist=True,
        )


def get_active_profile() -> tuple[str, str] | None:
    """Return the active ``(game-table, side)`` profile, or ``None``."""
    return _profile.value


def clear_active_profile() -> None:
    """Forget the active profile (test helper; persisted YAML is untouched)."""
    _profile.value = None


def load_persisted_profile() -> tuple[str, str] | None:
    """Restore and activate the last persisted profile, if any.

    Used on the ``--no-calibrate`` path so a headless run reuses the profile
    (and therefore the scoped values) picked during an earlier interactive
    setup.  Returns the activated profile, or ``None`` when nothing is stored.
    """
    entry = _store.get(_ACTIVE_PROFILE_KEY, None, persisted=True)
    if isinstance(entry, dict) and "table" in entry and "side" in entry:
        set_active_profile(entry["table"], entry["side"], persist=False)
        return _profile.value
    return None


# ---------------------------------------------------------------------------
# Parameter descriptors
# ---------------------------------------------------------------------------


class NumberParam:
    """A numeric mission parameter, declared once as a :class:`ParamSet` field.

    The descriptor owns the value's metadata (default, unit, persistence) and,
    via ``__set_name__``, its own storage key — so callers interact with the
    *attribute*, never a string::

        class P(ParamSet):
            cube_offset = NumberParam(default=0.0, unit="cm")


        P.cube_offset.get()  # -> float
        P.cube_offset.ask("Offset...")  # -> Step for the setup mission

    Args:
        default: Value returned by :meth:`get` before anything is entered.
        unit: Display unit shown on the input screen (e.g. ``"cm"``).
        persist: When ``True``, entered values are mirrored to
            ``racoon.calibration.yml`` and reused across restarts /
            ``--no-calibrate``.
        scoped: When ``True``, the value is stored *per active profile*
            (game-table + side) rather than globally — see
            :func:`configure_mission`.  Combine with ``persist=True`` so saved
            per-profile values survive a restart and can be reused.
        key: Explicit storage key.  Normally omitted — the attribute name is
            used automatically.  Only needed when declaring a parameter
            outside a class body (e.g. a module-level constant).
    """

    def __init__(
        self,
        default: float = 0.0,
        *,
        unit: str = "",
        persist: bool = False,
        scoped: bool = False,
        key: str | None = None,
    ) -> None:
        self._key = key
        self.default = float(default)
        self.unit = unit
        self.persist = persist
        self.scoped = scoped

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

    def _effective_key(self) -> str:
        """Storage key, namespaced by the active profile when ``scoped``.

        A scoped parameter with an active profile stores under
        ``"<table>::<side>::<key>"``; without a profile (or when un-scoped) it
        falls back to the plain :attr:`key`, so it behaves exactly like a
        global parameter until a profile is selected.
        """
        if self.scoped:
            profile = get_active_profile()
            if profile is not None:
                table, side = profile
                return f"{table}::{side}::{self.key}"
        return self.key

    def get(self) -> float:
        """Return the current value, or :attr:`default` if none was entered."""
        return float(_store.get(self._effective_key(), self.default, persisted=self.persist))

    def set(self, value: float) -> None:
        """Set the value directly."""
        _store.set(self._effective_key(), float(value), persist=self.persist)

    def is_set(self) -> bool:
        """Whether a value has been entered/persisted (vs. falling back to default)."""
        return _store.has(self._effective_key(), persisted=self.persist)

    def ask(self, prompt: str, *, title: str = "Setup") -> "Step":
        """Return a setup step that asks the operator for this value.

        The input screen is pre-filled with the current value (persisted or
        default) and shows the declared unit.  Cancelling keeps the existing
        value.  Under ``--no-calibrate`` the UI is skipped and the
        persisted/default value is used as-is.

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
    entered ``float``, or ``None`` if cancelled.  The physical robot button
    confirms (= Submit).
    """

    _primary_button_id = "submit"

    def __init__(
        self,
        prompt: str,
        *,
        initial_value: float = 0.0,
        unit: str = "",
        title: str = "Setup",
    ) -> None:
        super().__init__()
        self.title = title
        self._prompt = prompt
        self._unit = unit
        self.value = initial_value
        self._input_str = ""  # raw keypad input; empty means "showing default"

    def build(self) -> Widget:
        # Keep the prompt at "medium" (not "title"): a long prompt rendered at
        # title size stretches the column and pushes the value + Submit button
        # off the bottom of the screen.
        left: list[Widget] = [
            Text(self._prompt, size="medium", bold=True),
            Spacer(16),
            NumericInput(
                id="value",
                value=self.value,
                unit=self._unit,
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
            self.value = float(self._input_str) if self._input_str else 0.0
        except ValueError:
            self.value = 0.0

        await self.refresh()

    @on_change("value")
    async def on_adjust(self, value: float):
        # +/- buttons on the NumericInput
        self.value = value
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
    current value and stores whatever the operator confirms back into the
    parameter.

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


# ---------------------------------------------------------------------------
# Profile-driven setup step
# ---------------------------------------------------------------------------


#: A ``(param, prompt)`` pair for :func:`configure_mission`.
ParamPrompt = tuple[NumberParam, str]


class _ProfileEntryScreen(UIScreen["tuple[str, str] | None"]):
    """Single keypad screen to enter a profile like ``1B``.

    Mirrors the distance-calibration ``Split`` layout — a ``NumericKeypad`` on
    the right and, on the left, the live entry field plus action buttons — so
    the operator enters the whole profile on one screen instead of two menus:

    * the number keys set the **game-table number** (any number — not
      range-checked),
    * the ``A`` / ``B`` buttons pick the **starting side**,
    * the big field echoes the combined entry (e.g. ``1B``).

    Returns ``(table, side)`` on submit.  There is no cancel path — the
    operator must enter a table number and side.  The physical robot button
    confirms (= Submit) once a number and a side are entered.

    No renderer change is required: this reuses the existing ``NumericKeypad``
    and ``Button`` widgets — the side is picked with two buttons rather than
    extra keypad glyphs.
    """

    _primary_button_id = "submit"

    def __init__(self, *, title: str = "Setup") -> None:
        super().__init__()
        self.title = title
        self._number_str = ""  # typed table-number digits
        self._side: str | None = None  # "A" / "B" once chosen

    def _is_valid(self) -> bool:
        return bool(self._number_str) and self._side is not None

    def _entry_label(self) -> str:
        return f"{self._number_str or '_'}{self._side or '_'}"

    def build(self) -> Widget:
        left: list[Widget] = [
            Text("Game table + side", size="medium", bold=True),
            Spacer(8),
            Text(self._entry_label(), size="title", bold=True),
            Spacer(4),
            Text("Enter table number, then side", size="small", muted=True),
            Spacer(16),
            Row(
                children=[
                    Button("side_a", "A", style="primary" if self._side == "A" else "secondary"),
                    Button("side_b", "B", style="primary" if self._side == "B" else "secondary"),
                ],
                spacing=12,
            ),
            Spacer(20),
            Button(
                "submit",
                "Submit",
                style="success",
                icon="check",
                disabled=not self._is_valid(),
            ),
        ]
        return Split(left=left, right=[NumericKeypad()], ratio=(5, 6))

    @on_keypad()
    async def on_key(self, key: str):
        if key == "back":
            self._number_str = self._number_str[:-1]
        elif key.isdigit():
            self._number_str += key  # "." is ignored
        await self.refresh()

    @on_click("side_a")
    async def on_side_a(self):
        self._side = "A"
        await self.refresh()

    @on_click("side_b")
    async def on_side_b(self):
        self._side = "B"
        await self.refresh()

    @on_click("submit")
    async def on_submit(self):
        # Guard so a premature physical-button press (routed here) or a
        # renderer that ignores ``disabled`` cannot submit a partial entry.
        if self._is_valid():
            self.close((self._number_str, self._side))


@dsl(hidden=True)
class ConfigureMission(UIStep):
    """Setup step: enter a game-table / side profile, then fill scoped params.

    Prefer the :func:`configure_mission` factory over constructing this
    directly.  At runtime it drives the full profile flow:

    1. shows one keypad screen where the operator types the table number and
       taps ``A`` / ``B`` (e.g. ``1B``),
    2. activates that ``(table, side)`` profile so every scoped parameter now
       resolves under it,
    3. if a *complete* set of saved values already exists for the profile,
       offers to reuse them (asking first) and skips the questions,
    4. otherwise asks for each parameter in turn and stores it under the
       profile.

    Under ``--no-calibrate`` the UI is skipped entirely: the last persisted
    profile is restored (so scoped values read back from YAML) and no
    questions are asked.
    """

    def __init__(
        self,
        params: list[ParamPrompt],
        *,
        title: str = "Setup",
    ) -> None:
        super().__init__()
        self._params = list(params)
        self._title = title

    def _generate_signature(self) -> str:
        profile = get_active_profile()
        where = f"{profile[0]}{profile[1]}" if profile else "unset"
        return f"ConfigureMission(profile={where})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if is_no_calibrate():
            profile = load_persisted_profile()
            if profile is not None:
                robot.info(
                    f"--no-calibrate: reusing profile {profile[0]}{profile[1]} " "(input skipped)"
                )
            else:
                robot.warn("--no-calibrate: no saved profile — scoped params fall back to defaults")
            return

        selection = await self.show(_ProfileEntryScreen(title=self._title))
        if selection is None:
            robot.warn("Profile entry cancelled — keeping previous profile")
            return
        table, side = selection

        set_active_profile(table, side)
        robot.info(f"[profile] active: table {table} / Side {side}")

        # Offer reuse only when the profile is complete — every scoped param
        # already has a stored value — so the operator is never asked to
        # confirm a partial set that would still need topping up.
        if self._params and all(param.is_set() for param, _ in self._params):
            reuse = await self.confirm(
                f"Saved values found for {table}{side}. Reuse them?",
                title=self._title,
                yes_label="Reuse",
                no_label="Re-enter",
            )
            if reuse:
                for param, _ in self._params:
                    robot.info(
                        f"[param] {param.key} = {param.get():g}{param.unit} "
                        f"(reused for {table}{side})"
                    )
                return

        for param, prompt in self._params:
            await self._ask_one(robot, param, prompt)

    async def _ask_one(self, robot: "GenericRobot", param: NumberParam, prompt: str) -> None:
        screen = _AskNumberScreen(
            prompt,
            initial_value=param.get(),
            unit=param.unit,
            title=self._title,
        )
        value = await self.show(screen)
        if value is None:
            robot.warn(f"{param.key}: input cancelled, keeping {param.get():g}{param.unit}")
            return
        param.set(value)
        robot.info(f"[param] {param.key} = {param.get():g}{param.unit} (entered via setup UI)")


@dsl(tags=["control", "params"])
def configure_mission(
    params: list[ParamPrompt],
    *,
    title: str = "Setup",
) -> "Step":
    """Enter a game-table / side profile, then fill the scoped parameters.

    One-stop setup step for missions whose parameters differ per physical game
    table and starting side.  It shows a single keypad screen where the
    operator types the table number and taps ``A`` / ``B`` (e.g. ``1B``),
    activates that profile (so every ``scoped=True`` parameter now reads and
    writes under it), and then either reuses a complete set of previously saved
    values (after confirming) or asks for each parameter in turn.

    Prerequisites:
        The parameters passed here should be declared with ``scoped=True`` (and
        usually ``persist=True`` so saved values survive a restart).  Un-scoped
        parameters are unaffected and can still be asked with :func:`ask`.

    Args:
        params: ``(param, prompt)`` pairs — the scoped parameters to fill and
            the question shown for each.
        title: Screen title for every screen in the flow. Defaults to
            ``"Setup"``.

    Returns:
        A :class:`ConfigureMission` step, ready to drop into a ``seq([...])``.

    Example::

        from raccoon import NumberParam, ParamSet, configure_mission


        class P(ParamSet):
            cube_offset = NumberParam(default=0.0, unit="cm", scoped=True, persist=True)
            ramp_speed = NumberParam(default=0.5, scoped=True, persist=True)


        seq(
            [
                configure_mission(
                    params=[
                        (P.cube_offset, "Cube-row offset"),
                        (P.ramp_speed, "Ramp approach speed"),
                    ],
                ),
            ]
        )
    """
    return ConfigureMission(params, title=title)
