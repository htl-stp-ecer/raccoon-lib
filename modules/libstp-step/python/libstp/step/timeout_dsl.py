"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: timeout.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .timeout import Timeout


class TimeoutBuilder(StepBuilder):
    """Builder for Timeout. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._step = _UNSET
        self._seconds = _UNSET

    def step(self, value: Step):
        self._step = value
        return self

    def seconds(self, value: Union[float, int]):
        self._seconds = value
        return self

    def _build(self):
        kwargs = {}
        if self._step is not _UNSET:
            kwargs['step'] = self._step
        if self._seconds is not _UNSET:
            kwargs['seconds'] = self._seconds
        return Timeout(**kwargs)


@dsl(tags=['control', 'timeout'])
def timeout(step: Step = _UNSET, seconds: Union[float, int] = _UNSET):
    """
    Wrap a step with a time limit, cancelling it if it runs too long.

    Executes the given step normally but enforces a maximum wall-clock
    duration. If the wrapped step completes within the budget, the
    timeout step finishes successfully. If the step exceeds the time
    limit, it is cancelled via ``asyncio.wait_for`` and an error is
    logged. Any exception raised by the wrapped step propagates
    normally.

    This is especially useful around blocking steps like
    ``motor_move_to`` or ``wait_for_button`` that could stall
    indefinitely if the hardware misbehaves.

    Args:
        step: The step to execute under a time constraint. Must be a valid ``Step`` (or ``StepProtocol``) instance.
        seconds: Maximum allowed execution time in seconds. Must be positive.

    Returns:
        A TimeoutBuilder (chainable via ``.step()``, ``.seconds()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step import timeout
        from libstp.step.motor import motor_move_to

        # Give the arm 5 seconds to reach position 300; cancel if stuck
        timeout(
            motor_move_to(robot.motor(2), position=300, velocity=800),
            seconds=5.0,
        )

        # Ensure operator presses button within 30 seconds
        timeout(wait_for_button(), seconds=30.0)
    """
    b = TimeoutBuilder()
    if step is not _UNSET:
        b._step = step
    if seconds is not _UNSET:
        b._seconds = seconds
    return b


__all__ = ['TimeoutBuilder', 'timeout']
