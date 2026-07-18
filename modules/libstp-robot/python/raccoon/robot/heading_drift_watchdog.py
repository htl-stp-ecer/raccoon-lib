"""Heading-drift watchdog: warn when the chassis rotates while nothing drives.

Between motion steps (servo-only phases, grabs) no controller ticks the drive,
so external forces — an arm pushing off a game piece, a collision — can rotate
the chassis on low-friction mecanum wheels without anything noticing. Later
motion steps then inherit the heading error, which shows up as "cursed" snap
corrections that are expensive to trace back.

This watchdog runs on its OWN daemon thread (never on the asyncio motion loop,
mirroring ``ContinuousLocalizationFusion``): it polls the firmware IMU heading
and, while the ``"drive"`` resource is NOT held by any step, accumulates the
rotation since the drive was released. Crossing the threshold logs a WARNING
with the drifted angle, then re-arms so continued rotation warns again.

Purely diagnostic — it never commands anything.
"""

from __future__ import annotations

import contextlib
import math
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from raccoon.robot.api import GenericRobot

#: rotation while idle that triggers a warning (rad); 0.052 rad = 3.0 deg
DRIFT_WARN_RAD: float = 0.052
#: poll period (s) — cheap: one IMU getter + one dict lookup per tick
POLL_PERIOD_S: float = 0.05


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class HeadingDriftWatchdog:
    def __init__(self, robot: "GenericRobot", imu, threshold_rad: float = DRIFT_WARN_RAD):
        self._robot = robot
        self._imu = imu
        self._threshold = threshold_rad
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="heading-drift-watchdog", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    # ---- thread body ---------------------------------------------------
    def _run(self) -> None:
        from raccoon.step.resource import get_resource_manager

        try:
            mgr = get_resource_manager(self._robot)
        except Exception:
            return

        baseline: float | None = None
        while not self._stop.is_set():
            try:
                drive_held = mgr.is_held("drive")
                if drive_held:
                    baseline = None  # a controller owns the chassis; not our business
                else:
                    heading = float(self._imu.get_heading())
                    if baseline is None:
                        baseline = heading
                    else:
                        drift = _wrap_pi(heading - baseline)
                        if abs(drift) >= self._threshold:
                            with contextlib.suppress(Exception):
                                self._robot.warn(
                                    f"Chassis rotated {math.degrees(drift):+.1f}° while no "
                                    f"motion step was active — external contact? "
                                    f"(arm pushing off a game piece / collision)"
                                )
                            baseline = heading  # re-arm: further rotation warns again
            except Exception:
                # IMU unavailable (mock/sim) or transient read error: back off,
                # never let the watchdog take the process down.
                baseline = None
            self._stop.wait(POLL_PERIOD_S)


def start_heading_drift_watchdog(robot: "GenericRobot") -> HeadingDriftWatchdog | None:
    """Start the watchdog, or return ``None`` when no IMU is available."""
    try:
        from raccoon.hal import IMU

        imu = IMU()
        imu.get_heading()  # probe once; raises on mock platforms without IMU
    except Exception:
        return None
    wd = HeadingDriftWatchdog(robot, imu)
    wd.start()
    return wd
