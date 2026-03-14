from typing import Any
from .. import Step, StepProtocol
from ..annotation import dsl_step


@dsl_step(tags=["control", "loop"])
class LoopForever(Step):
    """Repeat a step indefinitely until externally cancelled.

    Wraps the given step in an infinite loop. Each iteration awaits the
    child step to completion before starting the next. The loop only
    terminates when the enclosing context cancels it (e.g. via
    ``do_while_active`` or ``do_until_checkpoint``).

    Args:
        step: The step to execute repeatedly. Must satisfy ``StepProtocol``.

    Example::

        from libstp.step.logic import loop_forever
        from libstp.step.timing import do_until_checkpoint

        # Continuously toggle a motor on and off until T=30s
        toggle = seq([
            motor_power(robot.motor(0), 100),
            wait(0.5),
            motor_off(robot.motor(0)),
            wait(0.5),
        ])
        do_until_checkpoint(30.0, loop_forever(toggle))
    """

    def __init__(self, step: StepProtocol):
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        self.step = step

    def collected_resources(self) -> frozenset[str]:
        return self.step.collected_resources()

    def _generate_signature(self) -> str:
        return "LoopForever()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        while True:
            await self.step.run_step(robot)


@dsl_step(tags=["control", "loop"])
class LoopFor(Step):
    """Repeat a step a fixed number of times.

    Wraps the given step in a counted loop. Each iteration awaits the
    child step to completion before starting the next. After all
    iterations complete, the step finishes normally.

    Args:
        step: The step to execute repeatedly. Must satisfy ``StepProtocol``.
        iterations: Number of times to run the step. Must be a positive
            integer.

    Example::

        from libstp.step.logic import loop_for

        # Drive forward and back 3 times
        loop_for(seq([drive_forward(20), drive_backward(20)]), iterations=3)
    """

    def __init__(self, step: StepProtocol, iterations: int):
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        if not isinstance(iterations, int) or iterations <= 0:
            raise ValueError(f"Iterations must be a positive integer, got {iterations}")

        self.step = step
        self.iterations = iterations

    def collected_resources(self) -> frozenset[str]:
        return self.step.collected_resources()

    def _generate_signature(self) -> str:
        return f"LoopFor(iterations={self.iterations})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        for _ in range(self.iterations):
            await self.step.run_step(robot)
