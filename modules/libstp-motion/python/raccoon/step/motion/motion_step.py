"""
Base class for motion steps with a unified fixed-rate update loop.

All motion steps share the same async timing pattern: dt calculation, near-zero
dt skip, sleep, and hard_stop cleanup. MotionStep owns this loop and exposes
on_start / on_update / on_stop lifecycle hooks for subclasses.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .. import Step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# dt-watchdog thresholds. A control tick whose dt runs far longer than the
# scheduled period means the asyncio loop was starved — typically a Pi
# undervoltage brownout stalling the stm32-data-reader's SPI sensor transfer
# (cross-confirmed by the reader's SLOW LOOP / SELF STALL and mcap battery-sag).
# While starved, this loop's `until` / `after_cm` guards cannot fire, so the
# robot keeps moving on the last command and OVERSHOOTS the stop condition.
# The watchdog is LOG-ONLY: it shouts, it never stops the step — behaviour is
# unchanged, so it is safe to leave enabled in competition.
_STALL_DT_FLOOR_S = 0.075  # ignore sub-75ms jitter (servo hiccups, GC)
_STALL_DT_FACTOR = 5.0  # ...and require dt > 5x the scheduled tick


@dataclass
class MotionLoopStats:
    """Summary of the fixed-rate loop timing observed during one motion step."""

    iterations: int
    elapsed_s: float
    avg_hz: float
    min_dt_ms: float
    max_dt_ms: float


class MotionStep(Step):
    """Base class for all motion steps. Handles the fixed-rate update loop."""

    hz: int = 100

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    def on_start(self, robot: "GenericRobot") -> None:
        """Called once before the loop. Override to set up motion/velocity."""

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        """Called each cycle with dt in seconds. Return True when motion is complete."""
        raise NotImplementedError

    def on_stop(self, robot: "GenericRobot") -> None:
        """Called after loop exits. Default: hard_stop."""
        robot.drive.hard_stop()

    async def _execute_step(self, robot: "GenericRobot") -> None:
        update_rate = 1 / self.hz

        self.on_start(robot)

        loop = asyncio.get_event_loop()
        last_time = loop.time() - update_rate  # seed so first dt ~= update_rate
        iterations = 0
        min_dt = float("inf")
        max_dt = 0.0
        start_time = loop.time()

        try:
            while True:
                current_time = loop.time()
                dt = max(current_time - last_time, 0.0)
                last_time = current_time

                if dt < 1e-4:
                    await asyncio.sleep(update_rate)
                    continue

                if dt > _STALL_DT_FLOOR_S and dt > _STALL_DT_FACTOR * update_rate:
                    self.error(
                        f"CONTROL-LOOP STALL: {type(self).__name__} tick "
                        f"dt={dt * 1000:.0f}ms (expected ~{update_rate * 1000:.0f}ms, "
                        f"{dt / update_rate:.0f}x) — asyncio loop starved; until/after_cm "
                        f"guards could not fire, robot moved UNCHECKED for this interval. "
                        f"Suspect Pi undervoltage/SPI stall. (log-only watchdog, not stopping)"
                    )

                min_dt = min(dt, min_dt)
                max_dt = max(dt, max_dt)
                iterations += 1

                if self.on_update(robot, dt):
                    break

                await asyncio.sleep(update_rate)
        finally:
            self.on_stop(robot)

            elapsed = loop.time() - start_time
            avg_hz = iterations / elapsed if elapsed > 0 else 0.0
            stats = MotionLoopStats(
                iterations=iterations,
                elapsed_s=elapsed,
                avg_hz=avg_hz,
                min_dt_ms=min_dt * 1000 if min_dt != float("inf") else 0.0,
                max_dt_ms=max_dt * 1000,
            )
            self.debug(
                f"Loop stats: {stats.iterations} iters, "
                f"{stats.elapsed_s:.2f}s, "
                f"{stats.avg_hz:.1f} Hz, "
                f"dt [{stats.min_dt_ms:.1f}, {stats.max_dt_ms:.1f}] ms"
            )
