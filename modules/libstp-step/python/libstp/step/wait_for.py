import asyncio
from typing import TYPE_CHECKING

from .base import Step
from .annotation import dsl_step
from .condition import StopCondition

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl_step(tags=["timing", "wait", "sensor"])
class WaitFor(Step):
    """Block until a stop condition is satisfied.

    Initializes the condition, then polls it at 50 Hz until it returns
    True. This lets you reuse any composable ``StopCondition`` — sensor
    triggers, timers, distance thresholds, or combinations — as a
    standalone wait without an accompanying motion.

    Args:
        condition: A ``StopCondition`` instance (or composition) to wait
            for.  Supports the same ``|``, ``&``, and ``+`` operators as
            motion ``.until()`` clauses.

    Example::

        from libstp.step import wait_for
        from libstp.step.condition import on_black, after_seconds

        # Wait until the front-left IR sensor sees black
        wait_for(on_black(Defs.front.left))

        # Wait for black OR a 5-second timeout
        wait_for(on_black(sensor) | after_seconds(5))
    """

    def __init__(self, condition: StopCondition) -> None:
        super().__init__()
        if not isinstance(condition, StopCondition):
            raise TypeError(
                f"Expected a StopCondition, got {type(condition).__name__}"
            )
        self._condition = condition

    def _generate_signature(self) -> str:
        return f"WaitFor({self._condition.__class__.__name__})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        self._condition.start(robot)
        while not self._condition.check(robot):
            await asyncio.sleep(0.02)
