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
    """
    Defer step construction until execution time.

    Wraps a factory callable that receives the robot instance and returns
    a step. The factory is called when the ``Defer`` step executes, not
    when the step tree is built. This allows steps to depend on runtime
    values such as sensor readings, odometry data, or results computed by
    earlier steps in a sequence.

    Args:
        factory: A callable that takes a ``GenericRobot`` and returns a
            ``Step`` to execute. Called exactly once when the deferred
            step runs.

    Returns:
        Defer: A step that lazily constructs and runs its child.

    Example::

        from libstp.step.logic import defer

        # Turn by an angle computed from a sensor reading at runtime
        seq([
            scan_step,
            defer(lambda robot: turn_left(
                compute_angle_from_scan(robot)
            )),
        ])

        # Drive a distance based on current odometry position
        defer(lambda robot: drive_forward(
            target_x - robot.odometry().getPosition().x
        ))
    """
    return Defer(factory)


@dsl(tags=["control", "run"])
def run(action: Callable[["GenericRobot"], Union[None, Awaitable[None]]]) -> Run:
    """
    Execute an arbitrary callable as a step.

    Wraps a sync or async callable so it can be used inline in a step
    sequence. This is useful for one-off side effects, logging,
    variable assignments, or any imperative code that does not warrant
    its own step class. The callable receives the robot instance and
    its return value is ignored (unless it returns an awaitable, which
    is then awaited).

    Args:
        action: A callable that takes a ``GenericRobot``. May be
            synchronous (returning ``None``) or asynchronous (returning
            an ``Awaitable[None]``).

    Returns:
        Run: A step that calls ``action`` when executed.

    Example::

        from libstp.step.logic import run

        # Emergency stop before turning
        seq([
            run(lambda robot: robot.drive.hard_stop()),
            turn_left(90),
        ])

        # Log odometry mid-sequence
        run(lambda robot: print(f"Heading: {robot.odometry().getHeading()}"))
    """
    return Run(action)
