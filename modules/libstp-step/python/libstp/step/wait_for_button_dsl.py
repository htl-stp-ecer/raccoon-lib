"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wait_for_button.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .wait_for_button import WaitForButton


class WaitForButtonBuilder(StepBuilder):
    """Builder for WaitForButton. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._message = 'Waiting for button press...'

    def message(self, value: str):
        self._message = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['message'] = self._message
        return WaitForButton(**kwargs)


@dsl(tags=['timing', 'button'])
def wait_for_button(message: str = 'Waiting for button press...'):
    """
    Wait for the operator to press the hardware button before continuing.

    Blocks the step sequence until the physical button on the robot
    controller is pressed. A message is displayed on the UI while
    waiting. This is useful for pausing between autonomous phases,
    confirming the robot is correctly positioned before starting, or
    gating destructive actions behind human approval.

    Args:
        message: Text displayed on screen while waiting for the button press. Defaults to ``"Waiting for button press..."``.

    Returns:
        A WaitForButtonBuilder (chainable via ``.message()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step import wait_for_button

        # Wait for the operator to confirm placement before starting
        sequence(
            wait_for_button("Place robot at starting position"),
            motor_power(robot.motor(0), 100),
            wait(3.0),
            motor_brake(robot.motor(0)),
        )
    """
    b = WaitForButtonBuilder()
    b._message = message
    return b


__all__ = ['WaitForButtonBuilder', 'wait_for_button']
