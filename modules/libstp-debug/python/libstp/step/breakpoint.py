from __future__ import annotations

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step
from typing import Any, Optional


class BreakpointStep(Step):
    def __init__(self, label: Optional[str] = None) -> None:
        """
        Lightweight breakpoint marker step.

        Args:
            label: Optional label that can be used for logging or UI display.
        """
        super().__init__()
        self._label = label

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Execute the breakpoint step. At runtime this simply logs the breakpoint hit.

        Args:
            device: The device to run on.
            definitions: Additional definitions needed for execution.
        """
        await super().run_step(device, definitions)
        label_suffix = f" ({self._label})" if self._label else ""
        self.info(f"Breakpoint reached{label_suffix}")


def breakpoint(label: Optional[str] = None) -> BreakpointStep:
    """
    Create a breakpoint step marker.

    Args:
        label: Optional label for the breakpoint.
    """
    return BreakpointStep(label=label)
