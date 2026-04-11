import asyncio
from typing import Union, TYPE_CHECKING

from . import Step, StepProtocol
from .annotation import dsl

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl(hidden=True)
class TimeoutOr(Step):
    """Run a step with a time limit; execute a fallback step if it times out."""

    _composite = True

    def __init__(self, step: Step, seconds: Union[float, int], fallback: Step) -> None:
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")
        if not isinstance(fallback, StepProtocol):
            raise TypeError(f"Expected fallback to be a Step instance, got {type(fallback)}")
        if seconds <= 0:
            raise ValueError(f"Timeout duration must be positive: {seconds}")

        self.step = step.resolve()
        self.seconds = float(seconds)
        self.fallback = fallback.resolve()

    def collected_resources(self) -> frozenset[str]:
        return self.step.collected_resources() | self.fallback.collected_resources()

    def _generate_signature(self) -> str:
        return (
            f"TimeoutOr(step={self.step.__class__.__name__}, "
            f"seconds={self.seconds:.3f}, "
            f"fallback={self.fallback.__class__.__name__})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        try:
            await asyncio.wait_for(self.step.run_step(robot), timeout=self.seconds)
        except asyncio.TimeoutError:
            self.warn(
                f"Step timed out after {self.seconds}s — running fallback "
                f"{self.fallback.__class__.__name__}"
            )
            await self.fallback.run_step(robot)


@dsl(tags=["control", "timeout"])
def timeout_or(step: Step, seconds: Union[float, int], fallback: Step) -> TimeoutOr:
    """Run a step with a time limit, executing a fallback step if it times out.

    Executes ``step`` normally and enforces a maximum wall-clock duration. If
    the step completes within the budget it finishes successfully and the
    fallback is never run. If ``step`` exceeds the time limit it is cancelled
    and ``fallback`` is executed in its place.

    This is useful when a motion step might stall (e.g., a drive that never
    reaches its target) and you want a recovery action rather than simply
    logging an error and moving on.

    Args:
        step: The primary step to execute under a time constraint. Must be a
            valid ``Step`` instance.
        seconds: Maximum allowed execution time in seconds. Must be positive.
        fallback: The step to execute when ``step`` times out.

    Returns:
        A TimeoutOr step instance.

    Example::

        from raccoon.step import timeout_or
        from raccoon.step.motion import drive_forward, drive_backward

        # Try driving forward 30 cm; if stuck after 3 s, back up 5 cm instead
        timeout_or(
            drive_forward(30),
            seconds=3.0,
            fallback=drive_backward(5),
        )
    """
    return TimeoutOr(step, seconds, fallback)
