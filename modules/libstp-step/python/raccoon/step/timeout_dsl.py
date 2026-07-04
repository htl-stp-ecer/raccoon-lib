"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: timeout.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .timeout import Timeout

from . import Step


class TimeoutBuilder(StepBuilder):
    """Builder for Timeout. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._step = _UNSET
        self._seconds = _UNSET
        self._propagate = False

    def step(self, value: Step):
        self._step = value
        return self

    def seconds(self, value: float | int):
        self._seconds = value
        return self

    def propagate(self, value: bool):
        self._propagate = value
        return self

    def _build(self):
        kwargs = {}
        if self._step is not _UNSET:
            kwargs["step"] = self._step
        if self._seconds is not _UNSET:
            kwargs["seconds"] = self._seconds
        kwargs["propagate"] = self._propagate
        return Timeout(**kwargs)


@dsl(tags=["control", "timeout"])
def timeout(step: Step = _UNSET, seconds: float | int = _UNSET, propagate: bool = False):
    """
    Wrap a step with a time limit, cancelling it if it runs too long.

    Executes the given step normally but enforces a maximum wall-clock
    duration. If the wrapped step completes within the budget, the
    timeout step finishes successfully. If the step exceeds the time
    limit, it is cancelled via ``asyncio.wait_for`` and an error is
    logged.

    By default (``propagate=False``) an expired timeout is *contained*: it
    cancels only the wrapped step and then finishes successfully, so the
    surrounding sequence and every remaining mission keep running. This is
    what you almost always want on a robot — one stuck ``motor_move_to``
    should not take down the whole run.

    Set ``propagate=True`` to instead re-raise ``TimeoutError`` after
    cancelling the wrapped step. That aborts the enclosing sequence/mission
    and is only appropriate when the wrapped step is a hard prerequisite for
    everything that follows.

    Note that the wrapped step is always cancelled on timeout (its
    ``finally`` blocks run, so a ``MotionStep`` still hard-stops the motors);
    ``propagate`` only controls whether the *timeout itself* is treated as a
    failure of the enclosing mission. Any *other* exception raised by the
    wrapped step propagates normally regardless of ``propagate``.

    This is especially useful around blocking steps like
    ``motor_move_to`` or ``wait_for_button`` that could stall
    indefinitely if the hardware misbehaves.

    Args:
        step: The step to execute under a time constraint. Must be a valid ``Step`` (or ``StepProtocol``) instance.
        seconds: Maximum allowed execution time in seconds. Must be positive.
        propagate: When ``False`` (default), a timeout cancels only the wrapped step and the mission continues. When ``True``, the ``TimeoutError`` is re-raised after cancellation, aborting the enclosing sequence/mission.

    Returns:
        A TimeoutBuilder (chainable via ``.step()``, ``.seconds()``, ``.propagate()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step import timeout
        from raccoon.step.motor import motor_move_to

        # Give the arm 5 seconds to reach position 300; cancel if stuck,
        # then carry on with the rest of the mission.
        timeout(
            motor_move_to(robot.motor(2), position=300, velocity=800),
            seconds=5.0,
        )

        # Hard prerequisite: abort the mission if the button isn't pressed
        timeout(wait_for_button(), seconds=30.0, propagate=True)
    """
    b = TimeoutBuilder()
    if step is not _UNSET:
        b._step = step
    if seconds is not _UNSET:
        b._seconds = seconds
    b._propagate = propagate
    return b


__all__ = ["TimeoutBuilder", "timeout"]
