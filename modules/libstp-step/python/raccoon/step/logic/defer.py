from __future__ import annotations

from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["control", "defer"])
class Defer(Step):
    """Defer step construction until execution time.

    Wraps a factory callable that receives the robot instance and returns
    a step. The factory is called when the ``Defer`` step executes, not
    when the step tree is built. This allows steps to depend on runtime
    values such as sensor readings, odometry data, or results computed by
    earlier steps in a sequence.

    Args:
        factory: A callable that takes a ``GenericRobot`` and returns a
            ``Step`` to execute. Called exactly once when the deferred
            step runs.

    Example::

        from raccoon.step.logic import defer

        # Turn by an angle computed from a sensor reading at runtime
        seq(
            [
                scan_step,
                defer(lambda robot: turn_left(compute_angle_from_scan(robot))),
            ]
        )
    """

    def __init__(self, factory: Callable[["GenericRobot"], Step]) -> None:
        super().__init__()
        if not callable(factory):
            msg = f"factory must be callable, got {type(factory).__name__}"
            raise TypeError(msg)
        self.factory = factory

    _composite = True

    def _generate_signature(self) -> str:
        return "Defer()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        step = self.factory(robot)
        await step.run_step(robot)


@dsl_step(tags=["control", "run"])
class Run(Step):
    """Execute an arbitrary callable as a step.

    Wraps a sync or async callable so it can be used inline in a step
    sequence. This is useful for one-off side effects, logging,
    variable assignments, or any imperative code that does not warrant
    its own step class. The callable receives the robot instance and
    its return value is ignored (unless it returns an awaitable, which
    is then awaited).

    Args:
        action: A callable that takes a ``GenericRobot`` and optionally
            returns an awaitable. Sync and async callables are both
            supported.

    Example::

        from raccoon.step.logic import run

        # Log the current heading between two drive steps
        seq(
            [
                drive_forward(25),
                run(lambda robot: print(f"Heading: {robot.odometry.get_heading()}")),
                drive_forward(25),
            ]
        )
    """

    def __init__(
        self,
        action: Callable[["GenericRobot"], None | Awaitable[None]],
    ) -> None:
        super().__init__()
        if not callable(action):
            msg = f"action must be callable, got {type(action).__name__}"
            raise TypeError(msg)
        self.action = action

    def _generate_signature(self) -> str:
        return "Run()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        result = self.action(robot)
        if result is not None:
            await result
