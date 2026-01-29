from __future__ import annotations

from typing import Any, Optional
from . import Step, dsl
from libstp.robot.api import GenericRobot

@dsl(hidden=True)
class BreakpointStep(Step):
    def __init__(self, label: Optional[str] = None) -> None:
        """
        Lightweight breakpoint marker step.

        Args:
            label: Optional label that can be used for logging or UI display.
        """
        super().__init__()
        self._label = label

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Execute the breakpoint step. At runtime this simply logs the breakpoint hit.
        """
        label_suffix = f" ({self._label})" if self._label else ""
        self.info(f"Breakpoint reached{label_suffix}")
        # ToDo: fully block process until lcm message has been sent to continue / unblock


@dsl(hidden=True)
def breakpoint(label: Optional[str] = None) -> BreakpointStep:
    """
    Create a breakpoint step marker.

    Args:
        label: Optional label for the breakpoint.
    """
    return BreakpointStep(label=label)
