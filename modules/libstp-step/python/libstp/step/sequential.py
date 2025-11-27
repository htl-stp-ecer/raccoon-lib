from typing import List, Optional

from . import Step, StepProtocol

class Sequential(Step):
    """
    Sequential step executor that runs steps in a sequential order.
    Each step will be executed only once.
    """

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

        self.steps: List[Step] = steps
        self._last_internal_step: Optional[Step] = steps[-1] if steps else None

    def _generate_signature(self) -> str:
        first = self.steps[0].__class__.__name__ if self.steps else "None"
        last = self.steps[-1].__class__.__name__ if self.steps else "None"
        return f"Sequential(count={len(self.steps)}, first={first}, last={last})"

    async def _execute_step(self, robot) -> None:
        """
        Execute each step in sequence, passing device and definitions to each step.
        Can only be run once.
        
        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution
            
        Raises:
            RuntimeError: If attempting to run this sequence more than once
        """
        # Now execute all child steps
        for i, step in enumerate(self.steps):
            await step.run_step(robot)

def seq(steps: List[Step]) -> Sequential:
    """Create a sequential sequence of steps"""
    return Sequential(steps)
