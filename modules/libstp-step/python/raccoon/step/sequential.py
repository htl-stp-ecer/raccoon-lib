from typing import List, Optional

from . import Step, StepProtocol, SimulationStep, SimulationStepDelta
from .base import _step_path
from .annotation import dsl


@dsl(hidden=True)
class Sequential(Step):
    """
    Composite step that runs child steps one after another.
    """

    _composite = True

    def __init__(self, steps: List[Step]) -> None:
        """
        Initialize Sequential step executor.
    
        Args:
            steps: List of Step objects to execute sequentially.
            
        Raises:
            TypeError: If any element in steps is not a Step instance.
        """
        super().__init__()
        
        if not isinstance(steps, list):
            raise TypeError(f"Expected steps to be a List[Step], got {type(steps)}")

        for i, step in enumerate(steps):
            if not isinstance(step, StepProtocol):
                raise TypeError(f"Element at index {i} is not a Step instance: {type(step)}")

        self.steps: List[Step] = [step.resolve() for step in steps]
        self._last_internal_step: Optional[Step] = self.steps[-1] if self.steps else None

    def collected_resources(self) -> frozenset[str]:
        result: set[str] = set()
        for step in self.steps:
            result |= step.collected_resources()
        return frozenset(result)

    def _generate_signature(self) -> str:
        first = self.steps[0].__class__.__name__ if self.steps else "None"
        last = self.steps[-1].__class__.__name__ if self.steps else "None"
        return f"Sequential(count={len(self.steps)}, first={first}, last={last})"

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        # Aggregate deltas from all child steps
        total_forward = 0.0
        total_strafe = 0.0
        total_angular = 0.0
        total_duration_ms = 0.0
        total_variance = 0.0
        for step in self.steps:
            child = step.to_simulation_step()
            total_forward += child.delta.forward
            total_strafe += child.delta.strafe
            total_angular += child.delta.angular
            total_duration_ms += child.average_duration_ms
            total_variance += child.duration_stddev_ms ** 2
        base.delta = SimulationStepDelta(
            forward=total_forward,
            strafe=total_strafe,
            angular=total_angular,
        )
        base.average_duration_ms = total_duration_ms
        base.duration_stddev_ms = total_variance ** 0.5
        return base

    async def _execute_step(self, robot) -> None:
        """
        Run each child step in order against the same robot instance.
        """
        total = len(self.steps)
        for i, step in enumerate(self.steps):
            token = self._push_path(f"{i + 1}/{total}")
            try:
                await step.run_step(robot)
            finally:
                _step_path.reset(token)

@dsl(hidden=True)
def seq(steps: List[Step]) -> Sequential:
    """Create a sequential composite from an explicit list of steps."""
    return Sequential(steps)
