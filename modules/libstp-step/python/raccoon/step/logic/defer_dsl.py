"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: defer.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .defer import Defer, Run

from .. import Step

class DeferBuilder(StepBuilder):
    """Builder for Defer. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._factory = _UNSET

    def factory(self, value: Callable[['GenericRobot'], Step]):
        self._factory = value
        return self

    def _build(self):
        kwargs = {}
        if self._factory is not _UNSET:
            kwargs['factory'] = self._factory
        return Defer(**kwargs)


@dsl(tags=['control', 'defer'])
def defer(factory: Callable[['GenericRobot'], Step] = _UNSET):
    """
    Defer step construction until execution time.

    Wraps a factory callable that receives the robot instance and returns
    a step. The factory is called when the ``Defer`` step executes, not
    when the step tree is built. This allows steps to depend on runtime
    values such as sensor readings, odometry data, or results computed by
    earlier steps in a sequence.

    Args:
        factory: A callable that takes a ``GenericRobot`` and returns a ``Step`` to execute. Called exactly once when the deferred step runs.

    Returns:
        A DeferBuilder (chainable via ``.factory()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.logic import defer

        # Turn by an angle computed from a sensor reading at runtime
        seq([
            scan_step,
            defer(lambda robot: turn_left(
                compute_angle_from_scan(robot)
            )),
        ])
    """
    b = DeferBuilder()
    if factory is not _UNSET:
        b._factory = factory
    return b


class RunBuilder(StepBuilder):
    """Builder for Run. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._action = _UNSET

    def action(self, value: Callable[['GenericRobot'], Union[None, Awaitable[None]]]):
        self._action = value
        return self

    def _build(self):
        kwargs = {}
        if self._action is not _UNSET:
            kwargs['action'] = self._action
        return Run(**kwargs)


@dsl(tags=['control', 'run'])
def run(action: Callable[['GenericRobot'], Union[None, Awaitable[None]]] = _UNSET):
    """
    Execute an arbitrary callable as a step.

    Wraps a sync or async callable so it can be used inline in a step
    sequence. This is useful for one-off side effects, logging,
    variable assignments, or any imperative code that does not warrant
    its own step class. The callable receives the robot instance and
    its return value is ignored (unless it returns an awaitable, which
    is then awaited).

    Args:
        action: A callable that takes a ``GenericRobot`` and optionally returns an awaitable. Sync and async callables are both supported.

    Returns:
        A RunBuilder (chainable via ``.action()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.logic import run

        # Log the current heading between two drive steps
        seq([
            drive_forward(25),
            run(lambda robot: print(f"Heading: {robot.odometry.get_heading()}")),
            drive_forward(25),
        ])
    """
    b = RunBuilder()
    if action is not _UNSET:
        b._action = action
    return b


__all__ = ['DeferBuilder', 'defer', 'RunBuilder', 'run']
