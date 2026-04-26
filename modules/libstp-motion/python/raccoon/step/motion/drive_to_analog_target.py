"""Drive until an analog sensor reaches its calibrated reference value.

Loads the target raw value stored by ``calibrate_analog_sensor()`` and drives
the robot forward or backward until the sensor reading crosses that threshold.
Direction is determined automatically at start time by comparing the current
reading to the calibrated target.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.hal import AnalogSensor
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["sensor", "drive"])
class DriveToAnalogTarget(MotionStep):
    """Drive until an analog sensor reaches its calibrated reference value.

    Reads the reference raw value stored by ``calibrate_analog_sensor()`` for
    the given sensor and set name, then drives forward or backward at the
    specified speed until ``sensor.read()`` crosses that threshold.  Direction
    is chosen automatically: if the current reading is below the target the
    robot drives forward (toward the target); if it is already above the
    target the robot drives backward.

    A ``timeout_cm`` can be provided as a safety backstop — the step stops
    after that distance even if the sensor has not reached the target.

    Prerequisites:
        ``calibrate_analog_sensor(sensor, set_name=...)`` must have been run
        (or a stored calibration must exist) before this step executes.
        Raises ``RuntimeError`` if no calibration data is found.

    Args:
        sensor: The analog sensor used to detect the target position.
        speed: Drive speed as a fraction of maximum (0.0–1.0, default 0.3).
            Use a slow speed for precise positioning.
        set_name: Which stored calibration point to target
            (default ``"default"``).
        timeout_cm: Maximum distance to drive in centimetres before giving
            up.  ``None`` disables the distance limit (default).

    Returns:
        A DriveToAnalogTarget instance.

    Raises:
        RuntimeError: If no calibration data exists for the given sensor and
            set name.

    Example::

        from raccoon.step.motion import drive_to_analog_target

        # Drive to the default calibrated ET-sensor position
        drive_to_analog_target(robot.defs.et_sensor)

        # Slower approach to a named position with 30 cm safety backstop
        drive_to_analog_target(
            robot.defs.et_sensor,
            speed=0.2,
            set_name="near",
            timeout_cm=30,
        )
    """

    _SENTINEL_DISTANCE_M = 5.0  # 5 m backstop when no timeout_cm given

    def __init__(
        self,
        sensor: "AnalogSensor",
        speed: float = 0.3,
        set_name: str = "default",
        timeout_cm: float | None = None,
    ) -> None:
        super().__init__()
        if not (0.0 < speed <= 1.0):
            msg = f"speed must be in (0.0, 1.0], got {speed}"
            raise ValueError(msg)
        self._sensor = sensor
        self._speed = speed
        self._set_name = set_name
        self._timeout_cm = timeout_cm
        self._target_value: float | None = None
        self._driving_forward: bool = True
        self._motion = None

    def _generate_signature(self) -> str:
        timeout_str = f", timeout={self._timeout_cm}cm" if self._timeout_cm else ""
        return (
            f"DriveToAnalogTarget(port={self._sensor.port}, "
            f"set={self._set_name!r}, speed={self._speed:.2f}{timeout_str})"
        )

    def on_start(self, robot: "GenericRobot") -> None:
        from raccoon.motion import LinearAxis, LinearMotion, LinearMotionConfig
        from raccoon.step.calibration.calibrate_analog_sensor import (
            ANALOG_SENSOR_STORE_SECTION,
            analog_sensor_store_key,
        )
        from raccoon.step.calibration.store import CalibrationStore

        store = CalibrationStore()
        key = analog_sensor_store_key(self._sensor, self._set_name)
        data = store.load(ANALOG_SENSOR_STORE_SECTION, key)
        if data is None:
            msg = (
                f"No analog sensor calibration found for port {self._sensor.port} "
                f"set '{self._set_name}'. "
                f"Run calibrate_analog_sensor() first."
            )
            raise RuntimeError(msg)

        self._target_value = float(data["target_value"])
        current = float(self._sensor.read())
        self._driving_forward = current < self._target_value

        self.debug(
            f"DriveToAnalogTarget: port={self._sensor.port} "
            f"current={current:.1f} target={self._target_value:.1f} "
            f"direction={'forward' if self._driving_forward else 'backward'}"
        )

        distance_m = (
            self._timeout_cm / 100.0 if self._timeout_cm is not None else self._SENTINEL_DISTANCE_M
        )
        sign = 1.0 if self._driving_forward else -1.0

        config = LinearMotionConfig()
        config.axis = LinearAxis.Forward
        config.distance_m = sign * distance_m
        config.speed_scale = self._speed
        self._motion = LinearMotion(robot.drive, robot.odometry, robot.motion_pid_config, config)
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        current = float(self._sensor.read())
        if self._driving_forward:
            if current >= self._target_value:
                return True
        elif current <= self._target_value:
            return True

        self._motion.update(dt)
        return self._motion.is_finished()
