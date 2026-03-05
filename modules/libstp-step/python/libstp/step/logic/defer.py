from typing import Callable, Awaitable, Union, TYPE_CHECKING

from .. import Step
from ..annotation import dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class Defer(Step):
    """Defers step creation until execution time.

    The factory receives the robot and returns the step to run.
    This allows steps to use runtime values (sensor readings, computed
    results from earlier steps, etc.) that aren't available at
    construction time.
    """

    def __init__(self, factory: Callable[["GenericRobot"], Step]) -> None:
        super().__init__()
        self.factory = factory

    async def _execute_step(self, robot: "GenericRobot") -> None:
        step = self.factory(robot)
        await step.run_step(robot)


@dsl(hidden=True)
class Run(Step):
    """Runs an arbitrary callable as a step.

    For simple side effects or computations that don't produce a child
    step.  The action can be sync or async.
    """

    def __init__(
        self,
        action: Callable[["GenericRobot"], Union[None, Awaitable[None]]],
    ) -> None:
        super().__init__()
        self.action = action

    async def _execute_step(self, robot: "GenericRobot") -> None:
        result = self.action(robot)
        if result is not None:
            await result


@dsl(tags=["control", "defer"])
def defer(factory: Callable[["GenericRobot"], Step]) -> Defer:
    """Create a step whose child step is built at execution time.

    Args:
        factory: Called with the robot instance; must return a Step.

    Example::

        seq([
            measure_step,
            defer(lambda robot: turn_left(compute_angle(measure_step, robot))),
        ])
    """
    return Defer(factory)


@dsl(tags=["control", "run"])
def run(action: Callable[["GenericRobot"], Union[None, Awaitable[None]]]) -> Run:
    """Create a step that executes an arbitrary callable.

    Args:
        action: Called with the robot instance.  May be sync or async.

    Example::

        seq([
            run(lambda robot: robot.drive.hard_stop()),
            turn_left(90),
        ])
    """
    return Run(action)
