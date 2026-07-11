from __future__ import annotations

import os
from abc import abstractmethod
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from raccoon.class_name_logger import ClassNameLogger

if TYPE_CHECKING:
    from raccoon.step.base import Step


@runtime_checkable
class MissionProtocol(Protocol):
    """Protocol for mission objects that can be run on a robot."""

    async def run(self, robot: "GenericRobot") -> None:
        """Execute the mission on the given robot."""
        ...


# A robot program runs many missions (setup → main missions → shutdown) in one
# process. Each ``Mission.run`` builds its own profiler from the same env, so a
# single fixed trace path would have every mission clobber the previous one —
# you'd only ever keep the last (usually the empty shutdown). Disambiguate the
# trace file per mission, keeping a per-name counter for missions that repeat.
_profile_run_counts: "dict[str, int]" = {}


def _per_mission_trace_path(trace_path: str, mission_name: str) -> str:
    from pathlib import Path

    n = _profile_run_counts.get(mission_name, 0)
    _profile_run_counts[mission_name] = n + 1
    tag = mission_name if n == 0 else f"{mission_name}-{n}"
    p = Path(trace_path)
    return str(p.with_name(f"{p.stem}.{tag}{p.suffix}"))


class Mission(ClassNameLogger, MissionProtocol):
    """Base mission that delegates execution to a root step sequence.

    Subclasses may set ``time_budget`` (seconds) as a class attribute to
    arm a per-mission deadline. If the mission exceeds its budget the
    robot's ``WatchdogManager`` fires, cancels the main-mission task, and
    routes through the normal shutdown path — the shutdown mission still
    runs. Subsequent missions do not run after a budget expiry.
    """

    time_budget: float | None = None

    def __str__(self):
        return self.__class__.__name__

    def __repr__(self):
        return self.__class__.__name__

    async def run(self, robot):
        """Build the mission sequence and execute it on the provided robot.

        Set the ``RACCOON_PROFILE`` environment variable to deep-profile the run
        (per-step phase breakdown + event-loop lag + a ``chrome://tracing``
        timeline) with zero code changes. Use ``1`` for the defaults, or a path
        to control where the trace lands::

            RACCOON_PROFILE=1 uv run raccoon run
            RACCOON_PROFILE=/tmp/mission.json uv run raccoon run

        Every knob is tunable via ``RACCOON_PROFILE_*`` env vars — see
        :meth:`raccoon.profiling.StepProfiler.from_env`.
        """
        self.info(f"Starting mission: {self.__class__.__name__}")
        root = self.sequence().resolve()

        # Profiling is an optional add-on living in a separate package
        # (``raccoon.profiling``, shipped with libstp-step). Only touch it when
        # the user actually asked for it, and degrade gracefully if the package
        # isn't installed on this target — a missing add-on must never crash a
        # mission run.
        profiler = None
        if os.environ.get("RACCOON_PROFILE"):
            try:
                from raccoon.profiling import StepProfiler
            except ImportError:
                self.warn(
                    "RACCOON_PROFILE is set but raccoon.profiling is not "
                    "installed — skipping profiling for this run"
                )
            else:
                profiler = StepProfiler.from_env()

        if profiler is not None:
            if profiler.trace_path:
                profiler.trace_path = _per_mission_trace_path(
                    profiler.trace_path, self.__class__.__name__
                )
            dest = profiler.trace_path or "(report only)"
            self.info(f"RACCOON_PROFILE set — profiling enabled, output → {dest}")
            async with profiler:
                await root.run_step(robot)
            if not profiler.trace_path:
                done = "report printed"
            elif profiler.defer_trace:
                # Buffered in RAM; the robot writes all traces after the run so
                # the json.dump never stalls the between-missions transition.
                done = f"trace buffered → {profiler.trace_path} (written post-run)"
            else:
                done = f"trace written to {profiler.trace_path}"
            self.info(f"Profiling finished — {done}")
        else:
            await root.run_step(robot)
        self.info(f"Completed mission: {self.__class__.__name__}")

    @abstractmethod
    def sequence(self) -> "Step":
        """Return the root step tree for this mission."""
        msg = "Method sequence() not implemented"
        raise NotImplementedError(msg)


class SetupMission(Mission):
    """Base class for setup missions with a customizable pre-start gate.

    Subclass this instead of ``Mission`` for setup missions. Override
    ``pre_start_gate()`` to customize what happens between the setup
    sequence and the main missions (e.g. skip waiting for light/button).

    Set ``setup_time`` (seconds) to display a countdown timer on every UI
    screen shown during the setup sequence.  The UI ticks the counter
    locally so no per-second LCM messages are required::

        class M000SetupMission(SetupMission):
            setup_time = 120  # 2-minute setup window

            def sequence(self) -> Step:
                return seq(calibrate_deadzone())

    Example::

        class MySetup(SetupMission):
            def sequence(self) -> Step:
                return seq(calibrate_deadzone())

            async def pre_start_gate(self, robot) -> None:
                # Skip the wait-for-light — just wait for button
                from raccoon.step import wait_for_button

                await wait_for_button().run_step(robot)


        class QuickSetup(SetupMission):
            def sequence(self) -> Step:
                return seq(calibrate_deadzone())

            async def pre_start_gate(self, robot) -> None:
                pass  # Skip all waiting — start immediately
    """

    #: Total seconds available for setup.  Shown as a countdown in every UI
    #: screen rendered during this mission.  Set to 0 to disable the timer.
    setup_time: int = 0

    _custom_pre_start_gate: bool = False

    @asynccontextmanager
    async def setup_timer_context(self):
        """Async context manager that activates the setup-timer.

        Spans the full setup phase — both the mission sequence and the
        pre-start gate — so the countdown is persistent across all steps
        and the WFL screen receives it too.  The robot calls this; user
        code never needs to touch it.

        When ``setup_time`` is 0 the context is a no-op.
        """
        if self.setup_time <= 0:
            yield
            return
        # Lazy import keeps mission module free of a hard UI dependency.
        from raccoon.ui.step import _active_setup_timer, _SetupTimerState  # type: ignore[import]

        token = _active_setup_timer.set(_SetupTimerState(self.setup_time))
        try:
            yield
        finally:
            _active_setup_timer.reset(token)

    async def pre_start_gate(self, robot) -> None:
        """Gate executed after the setup sequence and before main missions.

        The default delegates to the robot's built-in wait-for-light /
        wait-for-button logic. Override this to customize or skip the
        gate entirely.

        To skip all waiting::

            async def pre_start_gate(self, robot) -> None:
                pass  # start immediately
        """

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if "pre_start_gate" in cls.__dict__:
            cls._custom_pre_start_gate = True
