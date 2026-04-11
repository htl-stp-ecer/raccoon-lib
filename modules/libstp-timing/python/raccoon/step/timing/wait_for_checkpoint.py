import asyncio
from typing import TYPE_CHECKING, Union

from ..annotation import dsl_step
from raccoon.ui import UIStep
from raccoon.ui.screen import UIScreen
from raccoon.ui.widgets import (
    Center,
    Column,
    HintBox,
    Icon,
    Spacer,
    Text,
    Widget,
)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class _CheckpointCountdownScreen(UIScreen[None]):
    """Countdown display shown while ``WaitForCheckpoint`` is pending."""

    title = "Checkpoint"

    def __init__(self, checkpoint_seconds: float) -> None:
        super().__init__()
        self.checkpoint_seconds = checkpoint_seconds
        self.remaining_seconds: float = checkpoint_seconds

    def build(self) -> Widget:
        remaining = max(0.0, self.remaining_seconds)
        minutes = int(remaining // 60)
        seconds = remaining - minutes * 60
        timer_text = f"{minutes}:{seconds:04.1f}" if minutes > 0 else f"{seconds:.1f} s"

        return Center(children=[
            Column(children=[
                Icon("hourglass_top", size=64, color="blue"),
                Spacer(16),
                Text("Waiting for checkpoint", size="large", align="center"),
                Spacer(8),
                Text(
                    f"Target: T = {self.checkpoint_seconds:.1f} s",
                    size="small",
                    muted=True,
                    align="center",
                ),
                Spacer(24),
                Text(timer_text, size="title", bold=True, color="blue", align="center"),
                Spacer(16),
                HintBox(
                    "The next step will begin when the timer reaches zero.",
                    icon="schedule",
                ),
            ], align="center"),
        ])


@dsl_step(tags=["timing", "sync"])
class WaitForCheckpoint(UIStep):
    """Wait until a mission-relative time checkpoint is reached.

    Pauses execution until the robot's global synchronizer clock reaches
    the specified number of seconds since mission start. While waiting,
    a full-screen countdown UI shows the remaining time. If the
    checkpoint time has already passed, the step returns immediately
    without showing any UI. This is useful for synchronizing actions to
    absolute times within a timed Botball run (e.g. "at T=20s, start
    collecting").

    Prerequisites:
        The robot must have a ``synchronizer`` configured. The synchronizer
        clock starts when the mission begins.

    Args:
        checkpoint_seconds: The mission-relative time (in seconds) to
            wait for. Must be non-negative.

    Example::

        from raccoon.step.timing import wait_for_checkpoint

        seq([
            drive_forward(50),
            # Ensure we don't start the next action before T=10s
            wait_for_checkpoint(10.0),
            pick_up_tribble(),
            # Gate the final action to T=25s
            wait_for_checkpoint(25.0),
            drive_to_bin(),
        ])
    """

    _UI_REFRESH_INTERVAL = 0.1

    def __init__(self, checkpoint_seconds: Union[float, int]) -> None:
        super().__init__()

        if checkpoint_seconds < 0:
            raise ValueError(f"Checkpoint duration cannot be negative: {checkpoint_seconds}")

        self.checkpoint_seconds = float(checkpoint_seconds)

    def _generate_signature(self) -> str:
        return f"WaitForCheckpoint(seconds={self.checkpoint_seconds:.1f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_checkpoints import is_no_checkpoints

        if is_no_checkpoints():
            await robot.synchronizer.wait_until_checkpoint(self.checkpoint_seconds)
            return

        delta = self.checkpoint_seconds - robot.synchronizer.get_time()
        if delta <= 0:
            await robot.synchronizer.wait_until_checkpoint(self.checkpoint_seconds)
            return

        screen = _CheckpointCountdownScreen(self.checkpoint_seconds)
        screen.remaining_seconds = delta

        wait_task = asyncio.create_task(
            robot.synchronizer.wait_until_checkpoint(self.checkpoint_seconds)
        )
        try:
            await self.display(screen)
            while not wait_task.done():
                remaining = self.checkpoint_seconds - robot.synchronizer.get_time()
                screen.remaining_seconds = max(0.0, remaining)
                await screen.refresh()
                await self.pump_events()
                await asyncio.sleep(min(self._UI_REFRESH_INTERVAL, max(remaining, 0.01)))
            await wait_task
        finally:
            if not wait_task.done():
                wait_task.cancel()
                try:
                    await wait_task
                except asyncio.CancelledError:
                    pass
            await self.close_ui()
