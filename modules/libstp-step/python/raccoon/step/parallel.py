from __future__ import annotations

import asyncio

from raccoon.robot.api import GenericRobot

from . import SimulationStep, SimulationStepDelta, Step, StepProtocol
from .annotation import dsl
from .base import _step_path
from .resource import validate_no_overlap
from .sequential import Sequential, seq


@dsl(hidden=True)
class Parallel(Step):
    """
    Composite step that runs branches concurrently and waits for all of them.
    """

    _composite = True

    def __init__(self, steps: list[Step]) -> None:
        """
        Initialize Parallel step executor.

        Args:
            steps: List of Step objects to execute in parallel.

        Raises:
            TypeError: If any element in steps is not a Step instance.
        """
        super().__init__()

        if not isinstance(steps, list):
            msg = f"Expected steps to be a List[Step], got {type(steps)}"
            raise TypeError(msg)

        for i, step in enumerate(steps):
            if not isinstance(step, StepProtocol):
                msg = f"Element at index {i} is not a Step instance: {type(step)}"
                raise TypeError(msg)

        self.steps: list[Step] = [step.resolve() for step in steps]
        self._last_completed_step: Step | None = None

        # Pre-execution resource conflict check
        validate_no_overlap(self.steps, context="Parallel")

    def collected_resources(self) -> frozenset[str]:
        result: set[str] = set()
        for step in self.steps:
            result |= step.collected_resources()
        return frozenset(result)

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
        Run every branch concurrently against the same robot instance.
        """
        if not self.steps:
            return

        completed_steps = []
        total = len(self.steps)

        async def step_callback(index: int, step: Step) -> None:
            token = self._push_path(f"P[{index + 1}/{total}]")
            try:
                await step.run_step(robot)
            finally:
                _step_path.reset(token)
            completed_steps.append(step)

        tasks = [asyncio.create_task(step_callback(i, step)) for i, step in enumerate(self.steps)]

        await asyncio.gather(*tasks)

        if completed_steps:
            self._last_completed_step = completed_steps[-1]


@dsl(hidden=True)
def parallel(*args) -> Parallel:
    """
    Create a parallel composite step.

    Args:
        *args: Each argument can be a step, a sequential composite, or a list
            of steps that should be wrapped into a sequential branch.

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
            msg = f"Expected arg to be a Step, Sequential, or List[Step], got {type(arg)}"
            raise TypeError(msg)

    return Parallel(steps)
