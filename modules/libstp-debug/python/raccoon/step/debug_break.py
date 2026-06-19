from __future__ import annotations

from typing import TYPE_CHECKING

from raccoon.ui import UIStep

from .annotation import dsl

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl(hidden=True)
class DebugBreakStep(UIStep):
    """Step-level debug-break marker.

    Behaviour depends on the global ``--debug`` flag
    (``LIBSTP_DEBUG=1``, set by ``raccoon run --debug``):

    * **Debug mode on** — pauses the mission and waits for a hardware
      button press before continuing, like an interactive breakpoint.
    * **Debug mode off** — emits a single log line and returns
      immediately, so the step is effectively a no-op in normal runs.
    """

    def __init__(self, label: str | None = None) -> None:
        """
        Initialize a lightweight debug-break marker step.

        Args:
            label: Optional label used for the runtime log message and
                the on-screen prompt shown in debug mode.
        """
        super().__init__()
        self._label = label

    def _generate_signature(self) -> str:
        return f"DebugBreakStep(label={self._label!r})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        In debug mode, block until the button is pressed. Otherwise emit
        a log line and return immediately.
        """
        from raccoon.debug import is_debug

        label_suffix = f" ({self._label})" if self._label else ""

        if not is_debug():
            self.info(f"Debug break reached{label_suffix} (debug off, skipping)")
            return

        self.info(f"Debug break reached{label_suffix} — waiting for button press")
        prompt = f"Debug break: {self._label}" if self._label else "Debug break reached"
        await self.wait_for_button(f"{prompt}\nPress button to continue")


@dsl(hidden=True)
def debug_break(label: str | None = None) -> DebugBreakStep:
    """
    Create a debug-break marker step.

    In debug mode (``raccoon run --debug``) the step pauses the mission
    and waits for a hardware button press. Without ``--debug`` it logs a
    line and returns immediately.

    Args:
        label: Optional label included in the debug-break log message and
            the on-screen prompt.
    """
    return DebugBreakStep(label=label)
