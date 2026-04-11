"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: setup_timer.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .setup_timer import PauseSetupTimer, StartSetupTimer, ResumeSetupTimer


class PauseSetupTimerBuilder(StepBuilder):
    """Builder for PauseSetupTimer. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()

    def _build(self):
        kwargs = {}
        return PauseSetupTimer(**kwargs)


@dsl(tags=["timing", "setup"])
def pause_setup_timer():
    """
    Freeze the setup-phase countdown at its current value.

    Stops the remaining-time clock displayed at the top of every setup
    UI screen.  Elapsed time really stops advancing — a later
    :func:`start_setup_timer` (or :func:`resume_setup_timer`) picks up
    from where this step left the clock.

    No-op when used outside a SetupMission.

    Returns:
        A PauseSetupTimerBuilder (chainable via , ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.setup_timer import pause_setup_timer, start_setup_timer

        # As the very first step of the setup sequence: idle until the
        # operator is ready, then kick off the real setup timer.
        sequence(
            pause_setup_timer(),
            wait_for_button(),
            start_setup_timer(),
            calibrate_deadzone(),
            ...
        )
    """
    b = PauseSetupTimerBuilder()
    return b


class StartSetupTimerBuilder(StepBuilder):
    """Builder for StartSetupTimer. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()

    def _build(self):
        kwargs = {}
        return StartSetupTimer(**kwargs)


@dsl(tags=["timing", "setup"])
def start_setup_timer():
    """
    Start (or restart) the setup-phase countdown from full duration.

    Resets the elapsed time to zero and unpauses the timer, so the
    operator gets the full :attr:`SetupMission.setup_time` from the
    moment this step runs.  Combine with :func:`pause_setup_timer` at
    the top of the setup sequence to defer the countdown until you are
    ready.

    No-op when used outside a SetupMission.

    Returns:
        A StartSetupTimerBuilder (chainable via , ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.setup_timer import pause_setup_timer, start_setup_timer

        sequence(
            pause_setup_timer(),
            wait_for_button(),       # idle here — timer stays frozen
            start_setup_timer(),     # full setup_time begins now
            calibrate_deadzone(),
        )
    """
    b = StartSetupTimerBuilder()
    return b


class ResumeSetupTimerBuilder(StepBuilder):
    """Builder for ResumeSetupTimer. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()

    def _build(self):
        kwargs = {}
        return ResumeSetupTimer(**kwargs)


@dsl(tags=["timing", "setup"])
def resume_setup_timer():
    """
    Resume a paused setup-phase countdown without resetting it.

    Unlike :func:`start_setup_timer`, this preserves the elapsed time
    so the countdown continues from wherever it was frozen.

    No-op when used outside a SetupMission, or when the timer is
    already running.

    Returns:
        A ResumeSetupTimerBuilder (chainable via , ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.setup_timer import pause_setup_timer, resume_setup_timer

        sequence(
            pause_setup_timer(),
            wait_for_button(),
            resume_setup_timer(),   # continue from the frozen value
        )
    """
    b = ResumeSetupTimerBuilder()
    return b


__all__ = [
    "PauseSetupTimerBuilder",
    "pause_setup_timer",
    "StartSetupTimerBuilder",
    "start_setup_timer",
    "ResumeSetupTimerBuilder",
    "resume_setup_timer",
]
