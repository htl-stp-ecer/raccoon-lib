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
    Create a step that runs another step indefinitely.

    Args:
        step: The step to execute in a loop.

    Returns:
        LoopForeverStep: The step that runs the provided step indefinitely.
    """
    return LoopForeverStep(step=step)

@dsl(tags=["control", "loop"])
def loop_for(step: StepProtocol, iterations: int) -> LoopForStep:
    """
    Create a step that runs another step a specified number of times.

    Args:
        step: The step to execute in a loop.
        iterations: Number of times to run the step.

    Returns:
        LoopForStep: The step that runs the provided step for the specified iterations.
    """
    return LoopForStep(step=step, iterations=iterations)