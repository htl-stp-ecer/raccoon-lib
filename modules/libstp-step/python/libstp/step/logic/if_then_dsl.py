"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: if_then.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .if_then import IfThen

from .. import StepProtocol


class IfThenBuilder(StepBuilder):
    """Builder for IfThen. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._condition = _UNSET
        self._then_step = _UNSET
        self._else_step = None

    def condition(self, value: Callable[["GenericRobot"], bool]):
        self._condition = value
        return self

    def then_step(self, value: StepProtocol):
        self._then_step = value
        return self

    def else_step(self, value: Optional[StepProtocol]):
        self._else_step = value
        return self

    def _build(self):
        kwargs = {}
        if self._condition is not _UNSET:
            kwargs["condition"] = self._condition
        if self._then_step is not _UNSET:
            kwargs["then_step"] = self._then_step
        kwargs["else_step"] = self._else_step
        return IfThen(**kwargs)


@dsl(tags=["control", "logic"])
def if_then(
    condition: Callable[["GenericRobot"], bool] = _UNSET,
    then_step: StepProtocol = _UNSET,
    else_step: Optional[StepProtocol] = None,
):
    """
    Conditionally run one of two steps based on a runtime predicate.

    Evaluates ``condition`` once when the step executes and runs
    ``then_step`` if it returns truthy, otherwise runs ``else_step``
    (or nothing, if ``else_step`` is not provided). The branch decision
    is made at runtime, so the predicate can read sensors, odometry, or
    any state computed by earlier steps in the sequence.

    Resource usage is reported as the union of both branches because
    either may execute. The condition itself is not allowed to launch
    long-running work — it should return a boolean quickly.

    Args:
        condition: A callable taking a ``GenericRobot`` and returning a boolean. Called exactly once when the step runs.
        then_step: The step to execute when ``condition`` returns True.
        else_step: Optional step to execute when ``condition`` returns False. If omitted, the step does nothing on the false branch.

    Returns:
        A IfThenBuilder (chainable via ``.condition()``, ``.then_step()``, ``.else_step()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.logic import if_then

        # Pick a branch based on a sensor reading at runtime
        if_then(
            lambda robot: robot.front_ir.read() > 500,
            drive_backward(20),
            drive_forward(20),
        )
    """
    b = IfThenBuilder()
    if condition is not _UNSET:
        b._condition = condition
    if then_step is not _UNSET:
        b._then_step = then_step
    b._else_step = else_step
    return b


__all__ = ["IfThenBuilder", "if_then"]
