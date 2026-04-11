"""Setup-timer control steps.

Lets a :class:`SetupMission` pause and (re)start the setup-phase
countdown that is displayed in the UI.  Useful when you want to start
the program, idle for a bit, and only begin the real setup timer once
you are physically ready.

Outside of a SetupMission — i.e. when there is no active setup-timer —
both steps are no-ops.
"""

from typing import TYPE_CHECKING

from . import Step
from .annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["timing", "setup"])
class PauseSetupTimer(Step):
    """Freeze the setup-phase countdown at its current value.

    Stops the remaining-time clock displayed at the top of every setup
    UI screen.  Elapsed time really stops advancing — a later
    :func:`start_setup_timer` (or :func:`resume_setup_timer`) picks up
    from where this step left the clock.

    No-op when used outside a SetupMission.

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

    def _generate_signature(self) -> str:
        return "PauseSetupTimer()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.ui.step import set_setup_timer_paused
        set_setup_timer_paused(True)


@dsl_step(tags=["timing", "setup"])
class StartSetupTimer(Step):
    """Start (or restart) the setup-phase countdown from full duration.

    Resets the elapsed time to zero and unpauses the timer, so the
    operator gets the full :attr:`SetupMission.setup_time` from the
    moment this step runs.  Combine with :func:`pause_setup_timer` at
    the top of the setup sequence to defer the countdown until you are
    ready.

    No-op when used outside a SetupMission.

    Example::

        from raccoon.step.setup_timer import pause_setup_timer, start_setup_timer

        sequence(
            pause_setup_timer(),
            wait_for_button(),       # idle here — timer stays frozen
            start_setup_timer(),     # full setup_time begins now
            calibrate_deadzone(),
        )
    """

    def _generate_signature(self) -> str:
        return "StartSetupTimer()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.ui.step import reset_setup_timer
        reset_setup_timer()


@dsl_step(tags=["timing", "setup"])
class ResumeSetupTimer(Step):
    """Resume a paused setup-phase countdown without resetting it.

    Unlike :func:`start_setup_timer`, this preserves the elapsed time
    so the countdown continues from wherever it was frozen.

    No-op when used outside a SetupMission, or when the timer is
    already running.

    Example::

        from raccoon.step.setup_timer import pause_setup_timer, resume_setup_timer

        sequence(
            pause_setup_timer(),
            wait_for_button(),
            resume_setup_timer(),   # continue from the frozen value
        )
    """

    def _generate_signature(self) -> str:
        return "ResumeSetupTimer()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.ui.step import set_setup_timer_paused
        set_setup_timer_paused(False)
