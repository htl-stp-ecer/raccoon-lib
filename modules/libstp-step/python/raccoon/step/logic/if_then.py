from typing import Callable, Optional, TYPE_CHECKING

from .. import Step, StepProtocol
from ..annotation import dsl_step
from ..base import _step_path
from ..resource import validate_no_overlap

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["control", "logic"])
class IfThen(Step):
    """Conditionally run one of two steps based on a runtime predicate.

    Evaluates ``condition`` once when the step executes and runs
    ``then_step`` if it returns truthy, otherwise runs ``else_step``
    (or nothing, if ``else_step`` is not provided). The branch decision
    is made at runtime, so the predicate can read sensors, odometry, or
    any state computed by earlier steps in the sequence.

    Resource usage is reported as the union of both branches because
    either may execute. The condition itself is not allowed to launch
    long-running work — it should return a boolean quickly.

    Args:
        condition: A callable taking a ``GenericRobot`` and returning a
            boolean. Called exactly once when the step runs.
        then_step: The step to execute when ``condition`` returns True.
        else_step: Optional step to execute when ``condition`` returns
            False. If omitted, the step does nothing on the false branch.

    Example::

        from raccoon.step.logic import if_then

        # Pick a branch based on a sensor reading at runtime
        if_then(
            lambda robot: robot.front_ir.read() > 500,
            drive_backward(20),
            drive_forward(20),
        )
    """

    def __init__(
        self,
        condition: Callable[["GenericRobot"], bool],
        then_step: StepProtocol,
        else_step: Optional[StepProtocol] = None,
    ) -> None:
        super().__init__()

        if not callable(condition):
            raise TypeError(
                f"condition must be callable, got {type(condition).__name__}"
            )
        if not isinstance(then_step, StepProtocol):
            raise TypeError(
                f"then_step must be a Step, got {type(then_step).__name__}"
            )
        if else_step is not None and not isinstance(else_step, StepProtocol):
            raise TypeError(
                f"else_step must be a Step or None, got {type(else_step).__name__}"
            )

        self.condition = condition
        self.then_step = then_step
        self.else_step = else_step

        branches = [then_step] if else_step is None else [then_step, else_step]
        validate_no_overlap(branches, context="IfThen")

    def collected_resources(self) -> frozenset[str]:
        result = self.then_step.collected_resources()
        if self.else_step is not None:
            result = result | self.else_step.collected_resources()
        return result

    _composite = True

    def _generate_signature(self) -> str:
        return "IfThen()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if self.condition(robot):
            token = self._push_path("If[then]")
            try:
                await self.then_step.run_step(robot)
            finally:
                _step_path.reset(token)
        elif self.else_step is not None:
            token = self._push_path("If[else]")
            try:
                await self.else_step.run_step(robot)
            finally:
                _step_path.reset(token)
