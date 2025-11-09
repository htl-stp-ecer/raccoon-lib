import asyncio
from typing import List, Any, Optional


from . import Step, StepProtocol
from .sequential import seq, Sequential


class Parallel(Step):
    """
    Parallel step executor that runs steps concurrently.
    Waits for all steps to complete before continuing.
    Each step will be executed only once.
    """

    def __init__(self, steps: List[Step]) -> None:
        """
        Initialize Parallel step executor.

        Args:
            steps: List of Step objects to execute in parallel.

        Raises:
            TypeError: If any element in steps is not a Step instance.
        """
        super().__init__()

        if not isinstance(steps, list):
            raise TypeError(f"Expected steps to be a List[Step], got {type(steps)}")

        for i, step in enumerate(steps):
            if not isinstance(step, StepProtocol):
                raise TypeError(f"Element at index {i} is not a Step instance: {type(step)}")

        self.steps: List[Step] = steps
        self._last_completed_step: Optional[Step] = None

    async def run_step(self, robot) -> None:
        """
        Execute all steps in parallel, waiting for all to complete.
        Can only be run once.

        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution

        Raises:
            RuntimeError: If attempting to run this sequence more than once
        """
        # Call parent's run_step which will check if this step has already run
        # and set the _has_run flag to True
        await super().run_step(robot)

        # If no steps, nothing to do
        if not self.steps:
            return

        # Create a shared state to track completion
        completed_count = 0
        total_steps = len(self.steps)
        completed_steps = []

        # Define a callback wrapper for each step
        async def step_callback(step):
            nonlocal completed_count

            # Run the step
            await step.run_step(robot)

            # Track completion atomically
            completed_steps.append(step)
            completed_count += 1

            # If this is not the last step to complete, call on_exit with None
            if completed_count < total_steps:
                step.call_on_exit(next_step=None)

        # Create tasks with callbacks
        tasks = [asyncio.create_task(step_callback(step)) for step in self.steps]

        # Wait for all tasks to complete
        await asyncio.gather(*tasks)

        # Save the last completed step for later use in call_on_exit
        if completed_steps:
            self._last_completed_step = completed_steps[-1]
def parallel(*args) -> Parallel:
    """
    Create a parallel sequence of steps.

    Args:
        *args: Each argument can be a Step, a Sequential step, or a list of Steps.

    Returns:
        Parallel: A Parallel step instance containing all specified steps.

    Raises:
        TypeError: If any argument is not a Step, Sequential, or List[Step].
    """
    steps = []

    for arg in args:
        if isinstance(arg, list) and all(isinstance(i, StepProtocol) for i in arg):
            # For a list of steps, wrap in a sequence
            steps.append(seq(arg))
        elif isinstance(arg, StepProtocol):
            if isinstance(arg, Sequential):
                # Allow passing a sequence directly
                steps.append(arg)
            else:
                # Wrap individual step in a sequence
                steps.append(seq([arg]))
        else:
            raise TypeError(f"Expected arg to be a Step, Sequential, or List[Step], got {type(arg)}")

    return Parallel(steps)
