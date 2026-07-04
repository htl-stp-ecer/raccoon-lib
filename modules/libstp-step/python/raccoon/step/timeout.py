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
    logged.

    By default (``propagate=False``) an expired timeout is *contained*: it
    cancels only the wrapped step and then finishes successfully, so the
    surrounding sequence and every remaining mission keep running. This is
    what you almost always want on a robot — one stuck ``motor_move_to``
    should not take down the whole run.

    Set ``propagate=True`` to instead re-raise ``TimeoutError`` after
    cancelling the wrapped step. That aborts the enclosing sequence/mission
    and is only appropriate when the wrapped step is a hard prerequisite for
    everything that follows.

    Note that the wrapped step is always cancelled on timeout (its
    ``finally`` blocks run, so a ``MotionStep`` still hard-stops the motors);
    ``propagate`` only controls whether the *timeout itself* is treated as a
    failure of the enclosing mission. Any *other* exception raised by the
    wrapped step propagates normally regardless of ``propagate``.

    This is especially useful around blocking steps like
    ``motor_move_to`` or ``wait_for_button`` that could stall
    indefinitely if the hardware misbehaves.

    Args:
        step: The step to execute under a time constraint. Must be a
            valid ``Step`` (or ``StepProtocol``) instance.
        seconds: Maximum allowed execution time in seconds. Must be
            positive.
        propagate: When ``False`` (default), a timeout cancels only the
            wrapped step and the mission continues. When ``True``, the
            ``TimeoutError`` is re-raised after cancellation, aborting the
            enclosing sequence/mission.

    Example::

        from raccoon.step import timeout
        from raccoon.step.motor import motor_move_to

        # Give the arm 5 seconds to reach position 300; cancel if stuck,
        # then carry on with the rest of the mission.
        timeout(
            motor_move_to(robot.motor(2), position=300, velocity=800),
            seconds=5.0,
        )

        # Hard prerequisite: abort the mission if the button isn't pressed
        timeout(wait_for_button(), seconds=30.0, propagate=True)
    """

    _composite = True

    def __init__(self, step: Step, seconds: float | int, propagate: bool = False) -> None:
        super().__init__()

        if not isinstance(step, StepProtocol):
            msg = f"Expected step to be a Step instance, got {type(step)}"
            raise TypeError(msg)

        if seconds <= 0:
            msg = f"Timeout duration must be positive: {seconds}"
            raise ValueError(msg)

        self.step = step.resolve()
        self.seconds = float(seconds)
        self.propagate = propagate
        self.result = None

    def collected_resources(self) -> frozenset[str]:
        return self.step.collected_resources()

    def _generate_signature(self) -> str:
        return (
            f"Timeout(step={self.step.__class__.__name__}, "
            f"seconds={self.seconds:.3f}, propagate={self.propagate})"
        )

    async def _execute_step(self, robot) -> None:
        """Run the wrapped step, cancelling it if it exceeds the budget.

        ``asyncio.wait_for`` cancels the wrapped step on timeout, gives it a
        chance to clean up via its ``finally`` blocks (so a motion step still
        hard-stops the motors), and then re-raises ``TimeoutError``.

        By default we swallow that ``TimeoutError`` after logging: the whole
        point of a contained timeout is to bound *this* step's runtime, not to
        tear down the surrounding mission. When ``propagate`` is set the caller
        has opted into hard-failure semantics, so we re-raise instead.
        """
        try:
            await asyncio.wait_for(
                self.step.run_step(robot),
                timeout=self.seconds,
            )
        except TimeoutError:
            if self.propagate:
                self.error(
                    f"{self.step.__class__.__name__} exceeded "
                    f"{self.seconds:.3f}s timeout — cancelling and propagating"
                )
                raise
            self.warn(
                f"{self.step.__class__.__name__} exceeded "
                f"{self.seconds:.3f}s timeout — cancelled; mission continues"
            )
