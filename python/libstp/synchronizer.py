import asyncio

from libstp.logging import info, warn


class Synchroniser:
    def __init__(self):
        self.start_time = None

    def start_recording(self):
        self.start_time = asyncio.get_event_loop().time()

    def get_time(self):
        return asyncio.get_event_loop().time() - self.start_time

    async def wait_until_checkpoint(self, checkpoint_seconds):
        delta = checkpoint_seconds - self.get_time()
        if delta < 0:
            warn(f"[Synchroniser] Scheduler has passed the checkpoint at {checkpoint_seconds} seconds, delta: {delta}")
            return

        info(f"[Synchroniser] Scheduler waiting until {checkpoint_seconds} seconds, delta: {delta}")
        await asyncio.sleep(delta)
        info(f"[Synchroniser] Scheduler has reached the checkpoint at {checkpoint_seconds} seconds")