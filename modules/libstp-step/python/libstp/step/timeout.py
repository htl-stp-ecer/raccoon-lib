import asyncio
from typing import Union

from . import Step, StepProtocol

class Timeout(Step):
    """
    A step that executes another step with a timeout limit.
    Provides handlers for different outcomes (completion, timeout, error).
    """

    def __init__(self,
                 step: Step,
                 timeout_seconds: Union[float, int]) -> None:
        """
        Initialize a Timeout step.
        
        Args:
            step: The step to execute with a timeout.
            timeout_seconds: Maximum execution time in seconds.
            
        Raises:
            TypeError: If step is not a Step instance.
            ValueError: If timeout_seconds is negative.
        """
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        if timeout_seconds <= 0:
            raise ValueError(f"Timeout duration must be positive: {timeout_seconds}")

        self.step = step
        self.timeout_seconds = float(timeout_seconds)
        self.result = None

    def _generate_signature(self) -> str:
        return (
            f"Timeout(step={self.step.__class__.__name__}, "
            f"seconds={self.timeout_seconds:.3f})"
        )

    async def _execute_step(self, robot) -> None:
        """
        Execute the wrapped step with a timeout.
        
        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution
            
        Returns:
            TimeoutResult: The result of the execution
        """
        try:
            # Try to execute the step with a timeout
            await asyncio.wait_for(
                self.step.run_step(robot),
                timeout=self.timeout_seconds
            )
        except asyncio.TimeoutError:
            self.error(f"Step timed out after {self.timeout_seconds} seconds")
        except Exception:
            raise
def timeout(step: Step, seconds: float) -> Timeout:
    """Apply a timeout to a step"""
    return Timeout(step=step, timeout_seconds=seconds)
