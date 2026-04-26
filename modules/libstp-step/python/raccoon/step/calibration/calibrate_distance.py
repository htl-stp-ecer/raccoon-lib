"""
Distance calibration step using the new UI library.

Supports two calibration modes:
- Runtime calibration: Apply measured ticks_to_rad for this run (best accuracy now)
- Persistent learning: Update YAML baseline using EMA (converges over multiple runs)
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from raccoon.log import warn
from raccoon.project_yaml import (
    find_project_root as _find_project_root_util,
)
from raccoon.project_yaml import (
    read_project_value,
    update_project_value,
)
from raccoon.step.annotation import dsl_step
from raccoon.ui.screens.distance import (
    DistanceConfirmScreen,
    DistanceDrivingScreen,
    DistanceMeasureScreen,
    DistancePrepareScreen,
)
from raccoon.ui.step import UIStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot
    from raccoon.sensor_ir import IRSensor


# Default EMA alpha for calibration persistence
# Higher = slower convergence, more stable baseline
# 0.7 means ~83% of correction absorbed after 5 calibrations
DEFAULT_EMA_ALPHA = 0.7

# Distance (cm) for additional IR sensor calibration drives
_SENSOR_CAL_DRIVE_CM = 50.0


class CalibrationRequiredError(Exception):
    """Raised when an operation requires calibration but none has been performed."""


# Runtime calibration flag, kept on a single module-level container so
# the toggle is a simple attribute write — no ``global`` statement at the
# call sites and no ``object.__setattr__`` indirection.
class _CalibrationState:
    """Holds the runtime "have we calibrated yet?" flag."""

    done: bool = False


def is_distance_calibrated() -> bool:
    """Check if distance calibration has been performed."""
    return _CalibrationState.done


def check_distance_calibration() -> None:
    """Log a warning when distance calibration has not been performed yet."""
    if not _CalibrationState.done:
        warn(
            "Distance calibration highly suggested. Run calibrate() or calibrate_distance() first."
        )


def reset_distance_calibration() -> None:
    """Reset the calibration flag (for testing)."""
    _CalibrationState.done = False


@dataclass
class DistanceCalibrationResult:
    """Result of distance calibration."""

    requested_distance_cm: float
    measured_distance_cm: float
    scale_factor: float


@dataclass
class PerWheelCalibration:
    """Result of per-wheel distance calibration."""

    motor_port: int
    old_ticks_to_rad: float
    new_ticks_to_rad: float
    delta_ticks: int
    ema_baseline: float = 0.0  # EMA-filtered value for YAML persistence
    motor_name: str | None = None  # Motor name in YAML definitions


def _load_stored_ir_calibration(
    ir_sensors: list,
    calibration_sets: list[str],
    debug_fn,
    warn_fn,
) -> None:
    """Apply stored IR thresholds from CalibrationStore to each sensor (used under --no-calibrate)."""
    from raccoon import calibration_store as CalibrationStore
    from raccoon.calibration_store import CalibrationType

    for set_name in calibration_sets:
        for sensor in ir_sensors:
            key = f"{set_name}_port{sensor.port}"
            if CalibrationStore.has_readings(CalibrationType.IR_SENSOR, key):
                readings = CalibrationStore.get_readings(CalibrationType.IR_SENSOR, key)
                sensor.setCalibration(readings[1], readings[0])
                debug_fn(
                    f"--no-calibrate: IR port {sensor.port} set '{set_name}': "
                    f"black={readings[1]:.1f} white={readings[0]:.1f}"
                )
            else:
                warn_fn(
                    f"--no-calibrate: no stored readings for IR port {sensor.port} set '{set_name}'"
                )


def _find_project_root(start_path: Path | None = None) -> Path | None:
    """Find project root by searching upward for raccoon.project.yml."""
    return _find_project_root_util(start_path)


def _update_yaml_calibration(
    results: list[PerWheelCalibration],
    project_root: Path | None = None,
) -> bool:
    """
    Update ticks_to_rad values, following !include refs.

    Args:
        results: List of calibration results with new_ticks_to_rad and motor_name set
        project_root: Path to project root (auto-detected if None)

    Returns:
        True if YAML was updated successfully, False otherwise
    """
    if project_root is None:
        project_root = _find_project_root()

    if project_root is None:
        return False

    updated = False
    for result in results:
        if result.motor_name and update_project_value(
            project_root,
            ["definitions", result.motor_name, "calibration", "ticks_to_rad"],
            result.new_ticks_to_rad,
        ):
            updated = True

    return updated


def _find_motor_name_by_port(config: dict, port: int) -> str | None:
    """Find motor definition name by port number.

    When *config* has a 'definitions' key, searches there.  Falls back to
    reading definitions from the project YAML (following !include refs) when
    the in-memory dict is empty or missing.
    """
    definitions = config.get("definitions", {})
    if not definitions:
        project_root = _find_project_root()
        if project_root is not None:
            definitions = read_project_value(project_root, ["definitions"], {})

    for name, definition in definitions.items():
        if (
            isinstance(definition, dict)
            and definition.get("type") == "Motor"
            and definition.get("port") == port
        ):
            return name
    return None


@dsl_step(tags=["calibration", "distance"])
class CalibrateDistance(UIStep):
    """Calibrate per-wheel distance estimation via encoder measurement.

    Drives the robot a known distance, then prompts the user to enter the
    actual measured distance. The step computes a corrected ``ticks_to_rad``
    value for each drive motor so that odometry matches real-world distances.

    The calibration operates in two modes simultaneously:

    - **Runtime**: Applies the measured ``ticks_to_rad`` directly for best
      accuracy during this run.
    - **Persistent**: Updates the YAML baseline using an exponential moving
      average (EMA) so the stored value converges toward the true value over
      multiple calibration runs.

    Args:
        distance_cm: Distance (in cm) the robot drives during calibration.
            Longer distances yield better accuracy.
        speed: Drive speed during the calibration runs, as a fraction of
            max speed in ``[0.0, 1.0]``. Lower speeds reduce wheel slip
            and usually produce more accurate calibration.
        calibrate_light_sensors: If ``True``, run IR sensor calibration
            after the distance calibration is confirmed.
        persist_to_yaml: If ``True``, write the EMA-filtered baseline to
            ``raccoon.project.yml`` so it persists across program runs.
        ema_alpha: EMA smoothing coefficient between 0.0 and 1.0. Higher
            values produce slower convergence but a more stable baseline.
        calibration_sets: List of named IR calibration surface sets (e.g.
            ``["default", "transparent"]``). Only used when
            ``calibrate_light_sensors`` is ``True``.
        exclude_ir_sensors: List of ``IRSensor`` instances to skip during
            IR calibration.

    Example::

        from raccoon.step.calibration import calibrate_distance

        # Distance-only calibration with defaults
        calibrate_distance()

        # Distance + IR sensor calibration
        calibrate_distance(
            distance_cm=50.0,
            calibrate_light_sensors=True,
            calibration_sets=["default", "transparent"],
        )
    """

    _SENSOR_RETRY_SAMPLE_SECONDS = 5.0

    def __init__(
        self,
        distance_cm: float = 30.0,
        speed: float = 1.0,
        calibrate_light_sensors: bool = False,
        persist_to_yaml: bool = True,
        ema_alpha: float = 0.7,
        calibration_sets: list[str] | None = None,
        exclude_ir_sensors: list["IRSensor"] | None = None,
    ) -> None:
        super().__init__()
        self.calibration_distance_cm = distance_cm
        self.speed = speed
        self.calibrate_light_sensors = calibrate_light_sensors
        self.persist_to_yaml = persist_to_yaml
        self.ema_alpha = ema_alpha
        self.calibration_sets = calibration_sets or ["default"]
        self.exclude_ir_sensors: list["IRSensor"] = exclude_ir_sensors or []
        self.result: DistanceCalibrationResult | None = None
        self.per_wheel_results: list[PerWheelCalibration] = []

    def _generate_signature(self) -> str:
        excluded = [s.port for s in self.exclude_ir_sensors]
        return (
            f"CalibrateDistance(distance_cm={self.calibration_distance_cm}, "
            f"speed={self.speed}, "
            f"light_sensors={self.calibrate_light_sensors}, "
            f"persist={self.persist_to_yaml}, sets={self.calibration_sets}, "
            f"exclude_ir_ports={sorted(excluded)})"
        )

    async def _sample_sensors(
        self,
        sensors: list["IRSensor"],
        stop_event: asyncio.Event,
    ) -> dict[int, list[float]]:
        """Sample IR sensors at 10ms intervals until stop_event is set."""
        samples: dict[int, list[float]] = {sensor.port: [] for sensor in sensors}
        while not stop_event.is_set():
            for sensor in sensors:
                samples[sensor.port].append(float(sensor.read()))
            await asyncio.sleep(0.01)
        return samples

    async def _sample_sensors_for_duration(
        self,
        sensors: list["IRSensor"],
        duration_seconds: float,
    ) -> dict[int, list[float]]:
        stop_event = asyncio.Event()
        sampling_task = asyncio.create_task(self._sample_sensors(sensors, stop_event))
        try:
            await asyncio.sleep(duration_seconds)
        finally:
            stop_event.set()
        return await sampling_task

    def _collect_sensor_data(
        self,
        ir_sensors: list["IRSensor"],
        per_sensor_samples: dict[int, list[float]],
    ) -> list["SensorCalibrationData"]:
        """Build per-sensor calibration data for the dashboard."""
        from raccoon.step.calibration.sensors.dataclasses import SensorCalibrationData

        sensor_data = []
        for sensor in ir_sensors:
            samples = [float(v) for v in per_sensor_samples.get(sensor.port, [])]
            try:
                black_threshold = float(sensor.blackThreshold)
                white_threshold = float(sensor.whiteThreshold)
            except Exception:
                black_threshold = 0.0
                white_threshold = 0.0

            sensor_data.append(
                SensorCalibrationData(
                    port=sensor.port,
                    samples=samples,
                    black_threshold=black_threshold,
                    white_threshold=white_threshold,
                    black_mean=float(getattr(sensor, "blackMean", 0.0)),
                    white_mean=float(getattr(sensor, "whiteMean", 0.0)),
                    black_std=float(getattr(sensor, "blackStdDev", 0.0)),
                    white_std=float(getattr(sensor, "whiteStdDev", 0.0)),
                )
            )
        return sensor_data

    async def _drive_and_sample(
        self,
        robot: "GenericRobot",
        ir_sensors: list["IRSensor"],
    ) -> dict[int, list[float]]:
        """Drive forward while sampling IR sensors, then passive-brake."""
        from raccoon.step.motion.drive import _drive_forward_uncalibrated

        driving_screen = DistanceDrivingScreen(_SENSOR_CAL_DRIVE_CM)
        await self._render_screen(driving_screen)

        stop_sampling = asyncio.Event()
        sampling_task = asyncio.create_task(self._sample_sensors(ir_sensors, stop_sampling))
        try:
            drive_step = _drive_forward_uncalibrated(_SENSOR_CAL_DRIVE_CM, speed=self.speed)
            await drive_step.run_step(robot)
        finally:
            stop_sampling.set()
            sensor_samples = await sampling_task

        for motor in robot.drive.get_motors():
            motor.set_speed(0)

        return sensor_samples

    async def _confirm_light_sensors(
        self,
        robot: "GenericRobot",
        ir_sensors: list["IRSensor"],
        initial_samples: dict[int, list[float]],
        set_name: str = "default",
    ) -> None:
        from raccoon import calibration_store as CalibrationStore
        from raccoon.calibration_store import CalibrationType
        from raccoon.step.calibration.sensors.ir_results_screen import IRResultsDashboardScreen

        if not ir_sensors:
            return

        [sensor.port for sensor in ir_sensors]
        samples = initial_samples

        if not any(samples.values()):
            self.warn("No sensor samples collected during drive; resampling now")
            samples = await self._drive_and_sample(robot, ir_sensors)

        while True:
            # Calibrate each sensor with its own samples
            for sensor in ir_sensors:
                values = samples.get(sensor.port, [])
                if values:
                    sensor.calibrate(values)

            sensor_data = self._collect_sensor_data(ir_sensors, samples)

            dashboard_result = await self.show(IRResultsDashboardScreen(sensors=sensor_data))
            if dashboard_result is None:
                self.warn("Sensor calibration dashboard dismissed; aborting")
                return

            if dashboard_result.confirmed:
                for sensor, data in zip(ir_sensors, sensor_data, strict=False):
                    sensor.setCalibration(data.black_threshold, data.white_threshold)
                    CalibrationStore.store_readings(
                        CalibrationType.IR_SENSOR,
                        data.white_threshold,
                        data.black_threshold,
                        set_name + f"_port{sensor.port}",
                    )
                return

            # Retry: drive forward again and resample
            samples = await self._drive_and_sample(robot, ir_sensors)

    async def _drive_and_sample_for_set(
        self,
        robot: "GenericRobot",
        ir_sensors: list["IRSensor"],
        set_name: str,
    ) -> None:
        """Drive forward and sample IR sensors for an additional calibration set."""
        from raccoon.step.motion.drive import _drive_forward_uncalibrated

        label = set_name.upper()
        proceed = await self.confirm(
            f"Place sensors on {label} surface, then confirm to drive.",
            title=f"Calibrate: {label}",
            yes_label="Drive",
            no_label="Skip",
        )
        if not proceed:
            self.debug(f"Skipping calibration set '{set_name}'")
            return

        driving_screen = DistanceDrivingScreen(_SENSOR_CAL_DRIVE_CM)
        await self._render_screen(driving_screen)

        stop_sampling = asyncio.Event()
        sampling_task = asyncio.create_task(self._sample_sensors(ir_sensors, stop_sampling))
        try:
            drive_step = _drive_forward_uncalibrated(_SENSOR_CAL_DRIVE_CM, speed=self.speed)
            await drive_step.run_step(robot)
        finally:
            stop_sampling.set()
            sensor_samples = await sampling_task

        # Passive brake – no active hold so motors won't twitch over time
        drive_motors = robot.drive.get_motors()
        for motor in drive_motors:
            motor.set_speed(0)

        await self._confirm_light_sensors(robot, ir_sensors, sensor_samples, set_name)

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Execute the distance calibration.

        The calibration flow:
        1. Show prepare screen - user places robot
        2. Record encoder positions, robot drives
        3. Show measure screen - user enters measured distance
        4. Calculate and apply calibration
        5. Optionally calibrate IR sensors after distance confirmation

        Args:
            robot: The robot instance for driving
        """
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            _CalibrationState.done = True
            self.info("--no-calibrate: skipping distance calibration, using stored values")
            if self.calibrate_light_sensors:
                from raccoon.sensor_ir import IRSensor as _IRSensor

                ir_sensors = [
                    s
                    for s in robot.defs.analog_sensors
                    if isinstance(s, _IRSensor) and s not in self.exclude_ir_sensors
                ]
                _load_stored_ir_calibration(
                    ir_sensors, self.calibration_sets, self.debug, self.warn
                )
            return

        from raccoon.foundation import MotorCalibration
        from raccoon.sensor_ir import IRSensor
        from raccoon.step.motion.drive import _drive_forward_uncalibrated

        # Get wheel radius from kinematics
        wheel_radius = robot.drive.get_wheel_radius()
        self.debug(f"Wheel radius: {wheel_radius:.4f}m")

        # Get drive motors from kinematics
        drive_motors = robot.drive.get_motors()
        if not drive_motors:
            self.error("No drive motors found in kinematics")
            return

        self.debug(f"Found {len(drive_motors)} drive motor(s)")

        # Get IR sensors if light sensor calibration is enabled
        ir_sensors: list[IRSensor] = []
        if self.calibrate_light_sensors:
            for sensor in robot.defs.analog_sensors:
                if isinstance(sensor, IRSensor) and sensor not in self.exclude_ir_sensors:
                    ir_sensors.append(sensor)
            if ir_sensors:
                self.debug(f"Will calibrate {len(ir_sensors)} IR sensor(s) during drive")
            else:
                self.warn("No IR sensors found in robot.defs.analog_sensors")

        while True:
            # Phase 1: Prepare - user places robot
            self.debug("Distance calibration: prepare phase")
            await self.show(DistancePrepareScreen(self.calibration_distance_cm))

            # Record encoder positions BEFORE drive
            encoder_before: dict[int, int] = {}
            for motor in drive_motors:
                encoder_before[motor.port] = motor.get_position()
                self.debug(f"Motor {motor.port} encoder before: {encoder_before[motor.port]}")

            # Phase 2: Driving - show animation while robot moves
            driving_screen = DistanceDrivingScreen(self.calibration_distance_cm)
            await self._render_screen(driving_screen)

            # Execute the drive (with concurrent sensor sampling if enabled)
            drive_step = _drive_forward_uncalibrated(self.calibration_distance_cm, speed=self.speed)
            sensor_samples: dict[int, list[float]] = {}

            if ir_sensors:
                stop_sampling = asyncio.Event()
                sampling_task = asyncio.create_task(self._sample_sensors(ir_sensors, stop_sampling))
                try:
                    await drive_step.run_step(robot)
                finally:
                    stop_sampling.set()
                    sensor_samples = await sampling_task
            else:
                await drive_step.run_step(robot)

            # Passive brake – no active hold so motors won't twitch over time
            for motor in drive_motors:
                motor.set_speed(0)

            # Record encoder positions AFTER drive
            encoder_after: dict[int, int] = {}
            for motor in drive_motors:
                encoder_after[motor.port] = motor.get_position()
                self.debug(f"Motor {motor.port} encoder after: {encoder_after[motor.port]}")

            # Phase 3: Measure - user enters measured distance
            self.debug("Distance calibration: measure phase")
            measured_cm = await self.show(DistanceMeasureScreen(self.calibration_distance_cm))

            measured_m = measured_cm / 100.0
            self.debug(f"Measured distance: {measured_m:.4f}m")

            # Calculate new ticks_to_rad for each motor
            self.per_wheel_results = []
            for motor in drive_motors:
                delta_ticks = encoder_after[motor.port] - encoder_before[motor.port]
                old_calibration = motor.get_calibration()

                if abs(delta_ticks) < 10:
                    self.warn(f"Motor {motor.port} had insufficient movement ({delta_ticks} ticks)")
                    continue

                theta_rad = measured_m / wheel_radius
                new_ticks_to_rad = theta_rad / abs(delta_ticks)

                self.per_wheel_results.append(
                    PerWheelCalibration(
                        motor_port=motor.port,
                        old_ticks_to_rad=old_calibration.ticks_to_rad,
                        new_ticks_to_rad=new_ticks_to_rad,
                        delta_ticks=delta_ticks,
                    )
                )

            if not self.per_wheel_results:
                self.warn("No motors were calibrated due to insufficient movement")
                continue

            # Calculate average scale factor
            scale_factors = [
                r.new_ticks_to_rad / r.old_ticks_to_rad
                for r in self.per_wheel_results
                if r.old_ticks_to_rad > 0
            ]
            avg_scale_factor = sum(scale_factors) / len(scale_factors) if scale_factors else 1.0

            # Phase 4: Confirm - show results
            confirm_result = await self.show(
                DistanceConfirmScreen(
                    requested=self.calibration_distance_cm,
                    measured=measured_cm,
                )
            )

            if confirm_result.confirmed:
                # Find project root for motor name lookup and persistence
                project_root: Path | None = None
                if self.persist_to_yaml:
                    project_root = _find_project_root()
                    if not project_root:
                        self.warn("Could not find raccoon.project.yml for persistence")

                # Apply calibration to each motor
                for result in self.per_wheel_results:
                    # Find motor name for YAML persistence (follows !include refs)
                    if project_root:
                        result.motor_name = _find_motor_name_by_port({}, result.motor_port)

                    for motor in drive_motors:
                        if motor.port == result.motor_port:
                            old_cal = motor.get_calibration()
                            new_cal = MotorCalibration()
                            new_cal.ticks_to_rad = result.new_ticks_to_rad
                            new_cal.vel_lpf_alpha = old_cal.vel_lpf_alpha
                            motor.set_calibration(new_cal)

                            self.debug(
                                f"Motor {motor.port}: {result.old_ticks_to_rad:.4e} → "
                                f"{result.new_ticks_to_rad:.4e} rad/tick"
                            )
                            break

                # Persist EMA baselines to YAML
                if self.persist_to_yaml and project_root:
                    if _update_yaml_calibration(self.per_wheel_results, project_root):
                        self.info("Updated ticks_to_rad baselines in raccoon.project.yml")
                    else:
                        self.warn("Failed to persist calibration to raccoon.project.yml")

                # Mark as calibrated
                _CalibrationState.done = True

                self.result = DistanceCalibrationResult(
                    requested_distance_cm=self.calibration_distance_cm,
                    measured_distance_cm=measured_cm,
                    scale_factor=avg_scale_factor,
                )
                self.debug(
                    f"Distance calibration applied: {len(self.per_wheel_results)} motor(s) calibrated"
                )

                if self.calibrate_light_sensors and ir_sensors:
                    # First set uses samples from the distance calibration drive
                    await self._confirm_light_sensors(
                        robot, ir_sensors, sensor_samples, self.calibration_sets[0]
                    )

                    # Remaining sets each get their own drive-and-sample cycle
                    for cal_set in self.calibration_sets[1:]:
                        await self._drive_and_sample_for_set(robot, ir_sensors, cal_set)
                return

            # User wants to retry - loop continues
