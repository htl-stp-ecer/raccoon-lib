from typing import Any
from .. import Step, StepProtocol
from ..annotation import dsl


@dsl(hidden=True)
class LoopForeverStep(Step):
    def __init__(self, step: StepProtocol):
        """
        Initialize a LoopForeverStep that runs another step indefinitely.

        Args:
            step: The step to execute in a loop.

        Raises:
            TypeError: If step is not a Step instance.
        """
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        self.step = step

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Run the step indefinitely.

        Args:
            robot: The robot to run on.
        """
        while True:
            await self.step.run_step(robot)

@dsl(hidden=True)
class LoopForStep(Step):
    def __init__(self, step: StepProtocol, iterations: int):
        """
        Initialize a LoopForStep that runs another step a specified number of times.

        Args:
            step: The step to execute in a loop.
            iterations: Number of times to run the step.

        Raises:
            TypeError: If step is not a Step instance.
            ValueError: If iterations is not a positive integer.
        """
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        if not isinstance(iterations, int) or iterations <= 0:
            raise ValueError(f"Iterations must be a positive integer, got {iterations}")

        self.step = step
        self.iterations = iterations

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Run the step for the specified number of iterations.

        Args:
            robot: The robot to run on.
        """
        for _ in range(self.iterations):
            await self.step.run_step(robot)

@dsl(tags=["control", "loop"])
def loop_forever(step: StepProtocol) -> LoopForeverStep:
    """
    Repeat a step indefinitely until externally cancelled.

    Wraps the given step in an infinite loop. Each iteration awaits the
    child step to completion before starting the next. The loop only
    terminates when the enclosing context cancels it (e.g. via
    ``do_while_active`` or ``do_until_checkpoint``).

    Args:
        step: The step to execute repeatedly. Must satisfy ``StepProtocol``.

    Returns:
        LoopForeverStep: A step that runs ``step`` in an infinite loop.

    Example::

        from libstp.step.logic import loop_forever
        from libstp.step.motor import set_motor_speed

        # Continuously toggle a motor on and off (until parent cancels)
        toggle = seq([
            set_motor_speed(0, 1000),
            wait(0.5),
            set_motor_speed(0, 0),
            wait(0.5),
        ])
        do_until_checkpoint(30.0, loop_forever(toggle))
    """
    return LoopForeverStep(step=step)

@dsl(tags=["control", "loop"])
def loop_for(step: StepProtocol, iterations: int) -> LoopForStep:
    """
    Repeat a step a fixed number of times.

    Wraps the given step in a counted loop. Each iteration awaits the
    child step to completion before starting the next. After all
    iterations complete, the step finishes normally.

    Args:
        step: The step to execute repeatedly. Must satisfy ``StepProtocol``.
        iterations: Number of times to run the step. Must be a positive
            integer.

    Returns:
        LoopForStep: A step that runs ``step`` exactly ``iterations`` times.

    Example::

        from libstp.step.logic import loop_for
        from libstp.step.motor import set_motor_speed

        # Drive forward in three short bursts
        burst = seq([
            set_motor_speed(0, 800),
            wait(0.3),
            set_motor_speed(0, 0),
            wait(0.2),
        ])
        loop_for(burst, 3)
    """
    return LoopForStep(step=step, iterations=iterations)