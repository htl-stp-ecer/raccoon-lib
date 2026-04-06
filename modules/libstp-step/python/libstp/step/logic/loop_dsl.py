"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: loop.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .loop import LoopForever, LoopFor

from .. import StepProtocol

class LoopForeverBuilder(StepBuilder):
    """Builder for LoopForever. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._step = _UNSET

    def step(self, value: StepProtocol):
        self._step = value
        return self

    def _build(self):
        kwargs = {}
        if self._step is not _UNSET:
            kwargs['step'] = self._step
        return LoopForever(**kwargs)


@dsl(tags=['control', 'loop'])
def loop_forever(step: StepProtocol = _UNSET):
    """
    Repeat a step indefinitely until externally cancelled.

    Wraps the given step in an infinite loop. Each iteration awaits the
    child step to completion before starting the next. The loop only
    terminates when the enclosing context cancels it (e.g. via
    ``do_while_active`` or ``do_until_checkpoint``).

    Args:
        step: The step to execute repeatedly. Must satisfy ``StepProtocol``.

    Returns:
        A LoopForeverBuilder (chainable via ``.step()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.logic import loop_forever
        from libstp.step.timing import do_until_checkpoint

        # Continuously toggle a motor on and off until T=30s
        toggle = seq([
            motor_power(robot.motor(0), 100),
            wait(0.5),
            motor_off(robot.motor(0)),
            wait(0.5),
        ])
        do_until_checkpoint(30.0, loop_forever(toggle))
    """
    b = LoopForeverBuilder()
    if step is not _UNSET:
        b._step = step
    return b


class LoopForBuilder(StepBuilder):
    """Builder for LoopFor. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._step = _UNSET
        self._iterations = _UNSET

    def step(self, value: StepProtocol):
        self._step = value
        return self

    def iterations(self, value: int):
        self._iterations = value
        return self

    def _build(self):
        kwargs = {}
        if self._step is not _UNSET:
            kwargs['step'] = self._step
        if self._iterations is not _UNSET:
            kwargs['iterations'] = self._iterations
        return LoopFor(**kwargs)


@dsl(tags=['control', 'loop'])
def loop_for(step: StepProtocol = _UNSET, iterations: int = _UNSET):
    """
    Repeat a step a fixed number of times.

    Wraps the given step in a counted loop. Each iteration awaits the
    child step to completion before starting the next. After all
    iterations complete, the step finishes normally.

    Args:
        step: The step to execute repeatedly. Must satisfy ``StepProtocol``.
        iterations: Number of times to run the step. Must be a positive integer.

    Returns:
        A LoopForBuilder (chainable via ``.step()``, ``.iterations()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.logic import loop_for

        # Drive forward and back 3 times
        loop_for(seq([drive_forward(20), drive_backward(20)]), iterations=3)
    """
    b = LoopForBuilder()
    if step is not _UNSET:
        b._step = step
    if iterations is not _UNSET:
        b._iterations = iterations
    return b


__all__ = ['LoopForeverBuilder', 'loop_forever', 'LoopForBuilder', 'loop_for']
