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

    def _build(self):
        kwargs = {}
        return WaitForButton(**kwargs)


@dsl(tags=['timing', 'button'])
def wait_for_button():
    """
    Wait for the operator to press the hardware button before continuing.

    Blocks the step sequence until the physical button on the robot
    controller is pressed. A "Waiting for button press..." message is
    displayed on the UI while waiting. This is useful for pausing
    between autonomous phases, confirming the robot is correctly
    positioned before starting, or gating destructive actions behind
    human approval.

    Returns:
        A WaitForButtonBuilder (chainable via ).

    Example::

        from libstp.step import wait_for_button

        # Wait for the operator to confirm placement before starting
        sequence(
            wait_for_button(),
            motor_power(robot.motor(0), 100),
            wait(3.0),
            motor_brake(robot.motor(0)),
        )
    """
    b = WaitForButtonBuilder()
    return b


__all__ = ['WaitForButtonBuilder', 'wait_for_button']
