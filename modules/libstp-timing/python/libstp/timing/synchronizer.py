import asyncio

from libstp.foundation import initialize_logging
from libstp.log import error, info, debug, warn


class Synchronizer:
    """Mission-relative clock used by checkpoint-aware timing steps."""

    def __init__(self):
        """Create a synchronizer with no active start time."""
        self.start_time = None

    def start_recording(self):
        """Capture the current event-loop time as checkpoint zero."""
        self.start_time = asyncio.get_event_loop().time()

    def get_time(self):
        """Return seconds elapsed since ``start_recording()`` was called."""
        return asyncio.get_event_loop().time() - self.start_time

    async def wait_until_checkpoint(self, checkpoint_seconds):
        """Sleep until the mission-relative checkpoint or log that it was missed."""
        delta = checkpoint_seconds - self.get_time()
        if delta < 0:
            warn(f"[Synchroniser] Scheduler has passed the checkpoint at {checkpoint_seconds} seconds, delta: {delta}")
            return

        info(f"[Synchroniser] Scheduler waiting until {checkpoint_seconds} seconds, delta: {delta}")
        await asyncio.sleep(delta)
        info(f"[Synchroniser] Scheduler has reached the checkpoint at {checkpoint_seconds} seconds")

    async def do_until_checkpoint(self, checkpoint_seconds, func, *args, **kwargs):
        """
        Execute a function until the specified checkpoint is reached.

        Args:
            checkpoint_seconds: The time in seconds to wait until the checkpoint.
            func: The function to execute.
            *args: Positional arguments for the function.
            **kwargs: Keyword arguments for the function.
        """
        delta = checkpoint_seconds - self.get_time()
        if delta < 0:
            warn(f"[Synchroniser] Scheduler has passed the checkpoint at {checkpoint_seconds} seconds, delta: {delta}")
            return

        info(f"[Synchroniser] Scheduler executing until {checkpoint_seconds} seconds, delta: {delta}")
        task = asyncio.create_task(func(*args, **kwargs))
        await asyncio.sleep(delta)

        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        info(f"[Synchroniser] Scheduler has reached the checkpoint at {checkpoint_seconds} seconds")
