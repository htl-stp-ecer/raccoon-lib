from __future__ import annotations

from typing import Any, Optional
from . import Step, dsl
from libstp.robot.api import GenericRobot

@dsl(hidden=True)
class BreakpointStep(Step):
    """Step-level breakpoint marker that currently only emits a log line."""

    def __init__(self, label: Optional[str] = None) -> None:
        """
        Initialize a lightweight breakpoint marker step.

        Args:
            label: Optional label used for the runtime log message.
        """
        super().__init__()
        self._label = label

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Emit a log message for the breakpoint and return immediately.
        """
        label_suffix = f" ({self._label})" if self._label else ""
        self.info(f"Breakpoint reached{label_suffix}")
        # ToDo: fully block process until lcm message has been sent to continue / unblock


@dsl(hidden=True)
def breakpoint(label: Optional[str] = None) -> BreakpointStep:
    """
    Create a breakpoint marker step.

    Args:
        label: Optional label included in the breakpoint log message.
    """
    return BreakpointStep(label=label)
