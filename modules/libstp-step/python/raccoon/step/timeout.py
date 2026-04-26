from __future__ import annotations

import asyncio

from . import Step, StepProtocol
from .annotation import dsl_step


@dsl_step(tags=["control", "timeout"])
class Timeout(Step):
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

    Example::

        from raccoon.step import timeout
        from raccoon.step.motor import motor_move_to

        # Give the arm 5 seconds to reach position 300; cancel if stuck
        timeout(
            motor_move_to(robot.motor(2), position=300, velocity=800),
            seconds=5.0,
        )

        # Ensure operator presses button within 30 seconds
        timeout(wait_for_button(), seconds=30.0)
    """

    _composite = True

    def __init__(self, step: Step, seconds: float | int) -> None:
        super().__init__()

        if not isinstance(step, StepProtocol):
            msg = f"Expected step to be a Step instance, got {type(step)}"
            raise TypeError(msg)

        if seconds <= 0:
            msg = f"Timeout duration must be positive: {seconds}"
            raise ValueError(msg)

        self.step = step.resolve()
        self.seconds = float(seconds)
        self.result = None

    def collected_resources(self) -> frozenset[str]:
        return self.step.collected_resources()

    def _generate_signature(self) -> str:
        return f"Timeout(step={self.step.__class__.__name__}, " f"seconds={self.seconds:.3f})"

    async def _execute_step(self, robot) -> None:
        """Run the wrapped step and propagate a hard failure on timeout.

        ``asyncio.wait_for`` cancels the wrapped step on timeout, gives it a
        chance to clean up via its ``finally`` blocks, and then re-raises
        ``TimeoutError``. We log a clear message at the boundary and let the
        error propagate — silently swallowing it would let the surrounding
        mission continue with the wrapped step in an undefined hardware state
        (motor still spinning, servo still moving), which defeats the purpose
        of a timeout.
        """
        try:
            await asyncio.wait_for(
                self.step.run_step(robot),
                timeout=self.seconds,
            )
        except TimeoutError:
            self.error(
                f"{self.step.__class__.__name__} exceeded "
                f"{self.seconds:.3f}s timeout — cancelling and propagating"
            )
            raise
