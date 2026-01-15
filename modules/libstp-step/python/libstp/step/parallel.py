import asyncio
from typing import List, Optional

from libstp.robot.api import GenericRobot

from . import Step, StepProtocol, SimulationStep, SimulationStepDelta
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

    def _generate_signature(self) -> str:
        return f"Parallel(groups={len(self.steps)})"

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        # Aggregate deltas from all parallel branches (all movements happen)
        total_forward = 0.0
        total_strafe = 0.0
        total_angular = 0.0
        max_duration_ms = 0.0
        max_stddev_ms = 0.0
        for step in self.steps:
            child = step.to_simulation_step()
            total_forward += child.delta.forward
            total_strafe += child.delta.strafe
            total_angular += child.delta.angular
            max_duration_ms = max(max_duration_ms, child.average_duration_ms)
            max_stddev_ms = max(max_stddev_ms, child.duration_stddev_ms)
        base.delta = SimulationStepDelta(
            forward=total_forward,
            strafe=total_strafe,
            angular=total_angular,
        )
        base.average_duration_ms = max_duration_ms
        base.duration_stddev_ms = max_stddev_ms
        return base

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Execute all steps in parallel, waiting for all to complete.
        Can only be run once.

        Args:
            robot: The robot instance on which to run the steps.

        Raises:
            RuntimeError: If attempting to run this sequence more than once
        """
        if not self.steps:
            return

        completed_count = 0
        total_steps = len(self.steps)
        completed_steps = []

        async def step_callback(step):
            nonlocal completed_count

            await step.run_step(robot)

            completed_steps.append(step)
            completed_count += 1

            if completed_count < total_steps:
                step.call_on_exit(next_step=None)

        tasks = [asyncio.create_task(step_callback(step)) for step in self.steps]

        await asyncio.gather(*tasks)

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
