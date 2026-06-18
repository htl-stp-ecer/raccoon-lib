from __future__ import annotations

from . import Step, StepProtocol
from .annotation import dsl
from .base import _step_path


@dsl(hidden=True)
class Sequential(Step):
    """
    Composite step that runs child steps one after another.
    """

    _composite = True

    def __init__(self, steps: list[Step]) -> None:
        """
        Initialize Sequential step executor.

        Args:
            steps: List of Step objects to execute sequentially.

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
        self._last_internal_step: Step | None = self.steps[-1] if self.steps else None

    def collected_resources(self) -> frozenset[str]:
        result: set[str] = set()
        for step in self.steps:
            result |= step.collected_resources()
        return frozenset(result)

    def _generate_signature(self) -> str:
        first = self.steps[0].__class__.__name__ if self.steps else "None"
        last = self.steps[-1].__class__.__name__ if self.steps else "None"
        return f"Sequential(count={len(self.steps)}, first={first}, last={last})"

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
def seq(steps: list[Step]) -> Sequential:
    """Create a sequential composite from an explicit list of steps."""
    return Sequential(steps)
