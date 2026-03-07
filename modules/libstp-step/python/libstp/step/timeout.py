import asyncio
from typing import Union

from . import Step, StepProtocol
from .annotation import dsl


@dsl(hidden=True)
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
        Run the wrapped step and log if it exceeds the timeout budget.
        """
        try:
            await asyncio.wait_for(
                self.step.run_step(robot),
                timeout=self.timeout_seconds
            )
        except asyncio.TimeoutError:
            self.error(f"Step timed out after {self.timeout_seconds} seconds")
        except Exception:
            raise


@dsl(tags=["control", "timeout"])
def timeout(step: StepProtocol, seconds: float) -> Timeout:
    """Wrap a step with a time limit, cancelling it if it runs too long.

    Executes the given step normally but enforces a maximum wall-clock
    duration. If the wrapped step completes within the budget, the
    timeout step finishes successfully. If the step exceeds the time
    limit, it is cancelled via ``asyncio.wait_for`` and an error is
    logged. Any exception raised by the wrapped step propagates
    normally.

    This is especially useful around blocking steps like
    ``motor_move_to`` or ``wait_for_button`` that could stall
    indefinitely if the hardware misbehaves.

    Args:
        step: The step to execute under a time constraint. Must be a
            valid ``Step`` (or ``StepProtocol``) instance.
        seconds: Maximum allowed execution time in seconds. Must be
            positive.

    Returns:
        A ``Timeout`` step that wraps the original step with the given
        time limit.

    Example::

        from libstp.step import timeout
        from libstp.step.motor import motor_move_to

        # Give the arm 5 seconds to reach position 300; cancel if stuck
        timeout(
            motor_move_to(robot.motor(2), position=300, velocity=800),
            seconds=5.0,
        )

        # Ensure operator presses button within 30 seconds
        timeout(wait_for_button(), seconds=30.0)
    """
    return Timeout(step=step, timeout_seconds=seconds)
