"""Continuous, automatic line-sensor fusion into the localization particle filter.

The C++ ``Localization`` background thread only *predicts* — it integrates the
odometry delta every tick and never looks at a sensor. All map-based *correction*
has historically come from an explicit ``localization.observe()`` call inside a
resync step (``find_line_resync`` / ``align_to_wall_resync``). That means during
ordinary driving the filter is pure dead-reckoning and only snaps back to the map
at the rare moments a resync step runs.

This service closes that gap: it runs as an always-on background loop that, every
tick, reads the robot's line sensors, builds one :class:`Observation` carrying a
:class:`SurfaceMeasurement` per sensor (detected / not-detected against the map),
and feeds it to the filter. The filter then continuously weights its particles
against the table map's line geometry — real Monte-Carlo localization, not just
dead-reckoning.

Resync steps are NOT replaced: they remain the manual "I am confident I am on
axis X at 210 cm" hint (a hard, axis-snapping pose observation). This service is
the soft, automatic, every-tick correction underneath them.

The fusion pose channel anchors weakly on the filter's OWN current estimate (NO
ground truth is ever used), so unobservable degrees of freedom (e.g. the along-a-
straight-line axis when only crossing perpendicular lines) stay pinned to
odometry instead of drifting, while the surface measurements pull the observable
axes onto the map. The C++ filter only resamples when its effective sample size
drops (``resample_effective_sample_ratio``), so feeding an observation every tick
does not deplete particles.
"""

from __future__ import annotations

import asyncio
import contextlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot
    from raccoon.robot.geometry import SensorPosition


class ContinuousLocalizationFusion:
    """Always-on background fuser of line sensors → localization.

    Args:
        robot: the robot; provides ``localization``, ``all_sensors()`` and the
            per-sensor :class:`SensorPosition` geometry.
        threshold: ``probabilityOfBlack()`` at/above which a sensor counts as
            "on a line" (matches the ``on_black`` convention, default 0.7).
        hz: fusion rate; defaults to the localization tick rate (100 Hz). Lower
            it to reduce CPU; it never needs to exceed the predict rate.
        anchor_sigma_cm / anchor_sigma_deg: the weak pose-prior sigma (the filter
            is told "you are roughly where you currently think you are"). Small
            enough to pin unobservable axes, large enough to let the line
            measurements move the observable ones. ~8 cm works well in practice.
        line_sigma_cm: per-line-measurement sigma handed to the map likelihood.
    """

    def __init__(
        self,
        robot: "GenericRobot",
        *,
        threshold: float = 0.7,
        hz: float = 100.0,
        anchor_sigma_cm: float = 8.0,
        anchor_sigma_deg: float = 8.0,
        line_sigma_cm: float = 1.0,
    ) -> None:
        self._robot = robot
        self._threshold = float(threshold)
        self._period_s = 1.0 / float(hz)
        self._anchor_sigma_m = anchor_sigma_cm / 100.0
        import math

        self._anchor_sigma_rad = math.radians(anchor_sigma_deg)
        self._line_sigma_cm = float(line_sigma_cm)
        self._sensors = self._discover_line_sensors(robot)

    @staticmethod
    def _discover_line_sensors(robot: "GenericRobot") -> list[tuple[object, "SensorPosition"]]:
        """All line sensors on the robot, paired with their body-frame offset.

        Duck-typed on ``probabilityOfBlack`` (the line-sensor read), which
        distance/other sensors don't have, so they are skipped — as are sensors
        with no registered :class:`SensorPosition` (the offset is required to
        project the measurement onto the map)."""
        all_sensors = {}
        getter = getattr(robot, "all_sensors", None)
        if callable(getter):
            try:
                all_sensors = getter() or {}
            except Exception:
                all_sensors = {}

        out: list[tuple[object, "SensorPosition"]] = []
        for sensor, pos in all_sensors.items():
            if pos is None or not hasattr(sensor, "probabilityOfBlack"):
                continue
            out.append((sensor, pos))
        return out

    @property
    def sensor_count(self) -> int:
        return len(self._sensors)

    def _build_observation(self):
        """One Observation: weak pose anchor on the current estimate + a line
        SurfaceMeasurement per sensor (detected booleans read live)."""
        from raccoon.foundation import Pose
        from raccoon.localization import Observation, SurfaceKind, SurfaceMeasurement

        loc = self._robot.localization
        cur = loc.get_pose()
        pose = Pose()
        pose.position = [float(cur.position[0]), float(cur.position[1]), 0.0]
        pose.heading = float(cur.heading)

        measurements = []
        for sensor, pos in self._sensors:
            try:
                detected = sensor.probabilityOfBlack() >= self._threshold
            except Exception:
                continue  # uncalibrated / transient read error → skip this tick
            measurements.append(
                SurfaceMeasurement(
                    SurfaceKind.LINE,
                    pos,  # SensorPosition exposes forward_cm / strafe_cm
                    detected=bool(detected),
                    sigma_cm=self._line_sigma_cm,
                )
            )

        obs = Observation(
            pose=pose,
            sigma=(self._anchor_sigma_m, self._anchor_sigma_m, self._anchor_sigma_rad),
        )
        obs.surface_measurements = measurements
        return obs

    def tick(self) -> bool:
        """Read sensors once and push the observation. Returns False (no-op) when
        there is no localization or no line sensors. Exposed for tests and manual
        stepping; the loop calls it every period."""
        loc = getattr(self._robot, "localization", None)
        if loc is None or not self._sensors:
            return False
        loc.observe(self._build_observation())
        return True

    async def run(self) -> None:
        """Fuse every ``period`` until cancelled. Never raises out of the loop —
        a single bad read must not kill localization."""
        if not self._sensors:
            return
        while True:
            with contextlib.suppress(Exception):
                self.tick()
            await asyncio.sleep(self._period_s)


def start_localization_fusion(robot: "GenericRobot", **kwargs) -> "asyncio.Task[None] | None":
    """Launch :class:`ContinuousLocalizationFusion` as a background task.

    Returns the task (so the caller can cancel it) or ``None`` if there is no
    running event loop or the robot has no line sensors. Safe to call once at
    robot start; idempotent guard is the caller's responsibility."""
    fusion = ContinuousLocalizationFusion(robot, **kwargs)
    if fusion.sensor_count == 0:
        return None
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        return None
    task = loop.create_task(fusion.run())
    task.add_done_callback(lambda t: t.cancelled() or t.exception())
    return task
