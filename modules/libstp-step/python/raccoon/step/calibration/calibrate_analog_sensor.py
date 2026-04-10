"""Analog sensor (ET/distance) position calibration step.

Captures a reference raw reading from any AnalogSensor at a user-defined
robot position so that ``drive_to_analog_target()`` can reproduce the same
sensor-distance during a mission.
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Optional, Tuple, TYPE_CHECKING

from raccoon.step.annotation import dsl_step

from .calibrate_step import CalibrateStep

if TYPE_CHECKING:
    from raccoon.hal import AnalogSensor
    from raccoon.robot.api import GenericRobot


# CalibrationStore section shared with drive_to_analog_target
ANALOG_SENSOR_STORE_SECTION = "analog-sensor"


def analog_sensor_store_key(sensor: "AnalogSensor", set_name: str) -> str:
    """Return the CalibrationStore set key for *sensor* / *set_name*."""
    return f"port{sensor.port}_{set_name}"


@dataclass
class AnalogSensorCalibration:
    """Reference reading captured during calibration."""
    target_value: float
    std: float
    sample_count: int


@dsl_step(tags=["calibration", "sensor"])
class CalibrateAnalogSensor(CalibrateStep[AnalogSensorCalibration]):
    """Capture a reference analog sensor reading at a target robot position.

    Guides the operator to position the robot at a reference location, then
    samples the analog sensor for a short period to derive a stable reference
    raw value.  The result is persisted in ``racoon.calibration.yml`` under the
    ``analog-sensor`` section and consumed at runtime by
    ``drive_to_analog_target()``.

    Typical use: position the robot next to an object, run this step during
    setup, then use ``drive_to_analog_target()`` in the mission to reliably
    drive to that sensor distance every run.

    Supports ``--no-calibrate``: loads the stored reference without running
    the interactive flow when the flag is active and data already exists.

    Prerequisites:
        The sensor must be passed directly.  No auto-discovery from
        ``robot.defs`` is performed.

    Args:
        sensor: The analog sensor to calibrate (e.g. an ``ETSensor``).
        set_name: Label for this calibration point (default ``"default"``).
            Use different names for multiple reference distances on the same
            sensor port.
        sample_duration: How long to sample the sensor in seconds.
            Longer durations produce more stable averages.

    Returns:
        A CalibrateAnalogSensor instance.

    Raises:
        RuntimeError: If the sensor has no valid reading during sampling.

    Example::

        from raccoon.step.calibration import calibrate_analog_sensor

        # Calibrate ET sensor at the default target position
        calibrate_analog_sensor(robot.defs.et_sensor)

        # Two named positions on the same sensor
        calibrate_analog_sensor(robot.defs.et_sensor, set_name="near")
        calibrate_analog_sensor(robot.defs.et_sensor, set_name="far")
    """

    def __init__(
        self,
        sensor: "AnalogSensor",
        set_name: str = "default",
        sample_duration: float = 3.0,
    ) -> None:
        super().__init__(
            store_section=ANALOG_SENSOR_STORE_SECTION,
            store_set=analog_sensor_store_key(sensor, set_name),
        )
        self._sensor = sensor
        self._set_name = set_name
        self._sample_duration = sample_duration

    def _generate_signature(self) -> str:
        return (
            f"CalibrateAnalogSensor(port={self._sensor.port}, "
            f"set={self._set_name!r}, duration={self._sample_duration:.1f}s)"
        )

    # ── CalibrateStep hooks ───────────────────────────────────────────────────

    async def _collect(
        self, robot: "GenericRobot"
    ) -> Optional[AnalogSensorCalibration]:
        from raccoon.ui.screens.analog_sensor import (
            AnalogSensorPositionScreen,
            AnalogSensorSamplingScreen,
        )

        # Phase 1: operator positions the robot
        await self.show(AnalogSensorPositionScreen(
            port=self._sensor.port,
            set_name=self._set_name,
        ))

        # Phase 2: sample the sensor while displaying a progress screen
        values: list[float] = []

        async def _do_sample() -> None:
            count = max(1, int(self._sample_duration / 0.01))
            for _ in range(count):
                values.append(float(self._sensor.read()))
                await asyncio.sleep(0.01)

        await self.run_with_ui(
            AnalogSensorSamplingScreen(
                port=self._sensor.port,
                duration=self._sample_duration,
            ),
            _do_sample(),
        )

        if not values:
            self.warn(f"No samples collected from sensor port {self._sensor.port}")
            return None

        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        std = variance ** 0.5

        self.debug(
            f"Sensor port {self._sensor.port}: "
            f"mean={mean:.1f}, std={std:.1f}, n={len(values)}"
        )
        return AnalogSensorCalibration(
            target_value=mean,
            std=std,
            sample_count=len(values),
        )

    async def _confirm(
        self,
        robot: "GenericRobot",
        calibration: AnalogSensorCalibration,
    ) -> Tuple[bool, AnalogSensorCalibration]:
        from raccoon.ui.screens.analog_sensor import AnalogSensorConfirmScreen

        result = await self.show(AnalogSensorConfirmScreen(
            port=self._sensor.port,
            set_name=self._set_name,
            target_value=calibration.target_value,
            std=calibration.std,
            sample_count=calibration.sample_count,
        ))

        if result is None or not result.confirmed:
            return False, calibration
        return True, calibration

    def _apply(
        self, robot: "GenericRobot", calibration: AnalogSensorCalibration
    ) -> None:
        # Nothing to push to hardware; the value lives in the store and is
        # loaded by drive_to_analog_target at motion time.
        self.debug(
            f"Analog sensor port {self._sensor.port} / set '{self._set_name}': "
            f"target={calibration.target_value:.1f} ± {calibration.std:.1f} "
            f"({calibration.sample_count} samples)"
        )

    def _serialize(self, calibration: AnalogSensorCalibration) -> dict:
        return {
            "target_value": calibration.target_value,
            "std": calibration.std,
            "sample_count": calibration.sample_count,
        }

    def _deserialize(self, data: dict) -> AnalogSensorCalibration:
        return AnalogSensorCalibration(
            target_value=float(data["target_value"]),
            std=float(data.get("std", 0.0)),
            sample_count=int(data.get("sample_count", 0)),
        )
