"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wait_for_seconds.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .wait_for_seconds import WaitForSeconds


class WaitForSecondsBuilder(StepBuilder):
    """Builder for WaitForSeconds. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._seconds = _UNSET

    def seconds(self, value: Union[float, int]):
        self._seconds = value
        return self

    def _build(self):
        kwargs = {}
        if self._seconds is not _UNSET:
            kwargs['seconds'] = self._seconds
        return WaitForSeconds(**kwargs)


@dsl(tags=['timing', 'wait'])
def wait_for_seconds(seconds: Union[float, int] = _UNSET):
    """
    Pause execution for a fixed number of seconds.

    Suspends the current step sequence for the specified duration using
    an async sleep. No hardware commands are issued during the wait, and
    other concurrent tasks (e.g. odometry updates) continue running
    normally. The wait duration is deterministic and is reflected
    accurately in simulation estimates.

    Args:
        seconds: Duration to pause in seconds. Must be non-negative. Passing 0 yields control to the event loop for one tick without any meaningful delay.

    Returns:
        A WaitForSecondsBuilder (chainable via ``.seconds()``).

    Example::

        from libstp.step import wait

        # Pause between two motor commands
        sequence(
            motor_power(robot.motor(0), 100),
            wait(2.5),
            motor_brake(robot.motor(0)),
        )

        # Brief yield to let sensors settle
        wait(0.1)
    """
    b = WaitForSecondsBuilder()
    if seconds is not _UNSET:
        b._seconds = seconds
    return b


__all__ = ['WaitForSecondsBuilder', 'wait_for_seconds']
