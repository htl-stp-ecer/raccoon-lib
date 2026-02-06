"""
Distance calibration step using the new UI library.

Supports two calibration modes:
- Runtime calibration: Apply measured ticks_to_rad for this run (best accuracy now)
- Persistent learning: Update YAML baseline using EMA (converges over multiple runs)
"""

import asyncio
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, TYPE_CHECKING

import yaml

from libstp.step.annotation import dsl
from libstp.ui.step import UIStep
from libstp.ui.screens.distance import (
    DistancePrepareScreen,
    DistanceDrivingScreen,
    DistanceMeasureScreen,
    DistanceConfirmScreen,
)
from libstp.log import warn

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot
    from libstp.sensor_ir import IRSensor


# Default EMA alpha for calibration persistence
# Higher = slower convergence, more stable baseline
# 0.7 means ~83% of correction absorbed after 5 calibrations
DEFAULT_EMA_ALPHA = 0.7


class CalibrationRequiredError(Exception):
    """Raised when an operation requires calibration but none has been performed."""
    pass


# Simple calibration flag (runtime only)
_calibrated: bool = False


def is_distance_calibrated() -> bool:
    """Check if distance calibration has been performed."""
    return _calibrated


def check_distance_calibration() -> None:
    """Raise CalibrationRequiredError if not calibrated."""
    if not _calibrated:
        warn("Distance calibration highly suggested. Run calibrate() or calibrate_distance() first.")


def reset_distance_calibration() -> None:
    """Reset the calibration flag (for testing)."""
    global _calibrated
    _calibrated = False


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
    motor_name: Optional[str] = None  # Motor name in YAML definitions


def _find_project_root(start_path: Optional[Path] = None) -> Optional[Path]:
    """Find project root by searching upward for raccoon.project.yml."""
    if start_path is None:
        try:
            start_path = Path.cwd()
        except (FileNotFoundError, OSError):
            return None

    current = start_path.resolve()
    while current != current.parent:
        if (current / "raccoon.project.yml").exists():
            return current
        current = current.parent
    return None


def _update_yaml_calibration(
    results: List[PerWheelCalibration],
    project_root: Optional[Path] = None,
) -> bool:
    """
    Update ticks_to_rad values in raccoon.project.yml using EMA baselines.

    Args:
        results: List of calibration results with ema_baseline and motor_name set
        project_root: Path to project root (auto-detected if None)

    Returns:
        True if YAML was updated successfully, False otherwise
    """
    if project_root is None:
        project_root = _find_project_root()

    if project_root is None:
        return False

    config_path = project_root / "raccoon.project.yml"
    if not config_path.exists():
        return False

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        if not isinstance(config, dict):
            return False

        definitions = config.get('definitions', {})
        updated = False

        for result in results:
            if result.motor_name and result.motor_name in definitions:
                motor_def = definitions[result.motor_name]
                if 'calibration' in motor_def:
                    motor_def['calibration']['ticks_to_rad'] = result.ema_baseline
                    updated = True

        if updated:
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)

        return updated
    except (yaml.YAMLError, OSError, KeyError):
        return False


def _find_motor_name_by_port(config: Dict, port: int) -> Optional[str]:
    """Find motor definition name by port number."""
    definitions = config.get('definitions', {})
    for name, definition in definitions.items():
        if isinstance(definition, dict) and definition.get('port') == port:
            return name
    return None


@dsl(hidden=True)
class CalibrateDistance(UIStep):
    """
    Step for calibrating robot distance estimation via per-wheel ticks_to_rad adjustment.

    Two calibration mechanisms:
    1. Runtime: Apply the measured ticks_to_rad directly for best accuracy this run
    2. Persistent: Update YAML baseline using EMA so it converges over multiple runs

    The EMA formula: new_baseline = old_baseline × α + measured × (1 - α)
    With α=0.7, after ~5 calibrations the baseline absorbs ~83% of any systematic error.
    """

    _SENSOR_RETRY_SAMPLE_SECONDS = 5.0

    def __init__(
        self,
        calibration_distance_cm: float = 30.0,
        calibrate_light_sensors: bool = False,
        persist_to_yaml: bool = True,
        ema_alpha: float = DEFAULT_EMA_ALPHA,
    ) -> None:
        """
        Initialize the distance calibration step.

        Args:
            calibration_distance_cm: Distance to drive for calibration (default 30cm)
            calibrate_light_sensors: If True, calibrate IR sensors after distance confirmation
            persist_to_yaml: If True, update raccoon.project.yml with EMA-filtered baseline
            ema_alpha: EMA coefficient for baseline updates (0.0-1.0, higher = slower convergence)
        """
        super().__init__()
        self.calibration_distance_cm = calibration_distance_cm
        self.calibrate_light_sensors = calibrate_light_sensors
        self.persist_to_yaml = persist_to_yaml
        self.ema_alpha = ema_alpha
        self.result: Optional[DistanceCalibrationResult] = None
        self.per_wheel_results: List[PerWheelCalibration] = []

    def _generate_signature(self) -> str:
        return f"CalibrateDistance(distance_cm={self.calibration_distance_cm}, light_sensors={self.calibrate_light_sensors}, persist={self.persist_to_yaml})"

    async def _sample_sensors(
        self,
        sensors: List["IRSensor"],
        stop_event: asyncio.Event,
    ) -> Dict[int, List[float]]:
        """Sample IR sensors at 10ms intervals until stop_event is set."""
        samples: Dict[int, List[float]] = {sensor.port: [] for sensor in sensors}
        while not stop_event.is_set():
            for sensor in sensors:
                samples[sensor.port].append(float(sensor.read()))
            await asyncio.sleep(0.01)
        return samples

    async def _sample_sensors_for_duration(
        self,
        sensors: List["IRSensor"],
        duration_seconds: float,
    ) -> Dict[int, List[float]]:
        stop_event = asyncio.Event()
        sampling_task = asyncio.create_task(self._sample_sensors(sensors, stop_event))
        try:
            await asyncio.sleep(duration_seconds)
        finally:
            stop_event.set()
        return await sampling_task

    async def _confirm_light_sensors(
        self,
        ir_sensors: List["IRSensor"],
        initial_samples: Dict[int, List[float]],
    ) -> None:
        from libstp.step.calibration.sensors.ir_calibrating_screen import IRCalibratingScreen
        from libstp.step.calibration.sensors.ir_confirm_screen import IRConfirmScreen

        if not ir_sensors:
            return

        ports = [sensor.port for sensor in ir_sensors]
        samples = initial_samples

        if not any(samples.values()):
            self.warn("No sensor samples collected during drive; resampling now")
            samples = await self.run_with_ui(
                IRCalibratingScreen(ports=ports),
                self._sample_sensors_for_duration(ir_sensors, self._SENSOR_RETRY_SAMPLE_SECONDS),
            )

        while True:
            display_sensor = next(
                (sensor for sensor in ir_sensors if samples.get(sensor.port)),
                ir_sensors[0],
            )
            self.debug(f"Using sensor port {display_sensor.port} for threshold confirmation")
            values = samples.get(display_sensor.port, [])
            success = bool(values) and display_sensor.calibrate(values)
            black_threshold = display_sensor.blackThreshold if success else 0.0
            white_threshold = display_sensor.whiteThreshold if success else 0.0

            confirm_result = await self.show(
                IRConfirmScreen(
                    black_threshold=black_threshold,
                    white_threshold=white_threshold,
                    collected_values=values,
                )
            )
            if confirm_result is None:
                self.warn("Sensor calibration confirmation dismissed; aborting")
                return

            if confirm_result.confirmed:
                for sensor in ir_sensors:
                    sensor.setCalibration(
                        confirm_result.black_threshold,
                        confirm_result.white_threshold,
                    )
                return

            samples = await self.run_with_ui(
                IRCalibratingScreen(ports=ports),
                self._sample_sensors_for_duration(ir_sensors, self._SENSOR_RETRY_SAMPLE_SECONDS),
            )

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
        from libstp.step.motion.drive import _drive_forward_uncalibrated
        from libstp.step.motion.stop import stop
        from libstp.foundation import MotorCalibration
        from libstp.sensor_ir import IRSensor

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
        ir_sensors: List[IRSensor] = []
        if self.calibrate_light_sensors:
            for sensor in robot.defs.analog_sensors:
                if isinstance(sensor, IRSensor):
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
            encoder_before: Dict[int, int] = {}
            for motor in drive_motors:
                encoder_before[motor.port] = motor.get_position()
                self.debug(f"Motor {motor.port} encoder before: {encoder_before[motor.port]}")

            # Phase 2: Driving - show animation while robot moves
            driving_screen = DistanceDrivingScreen(self.calibration_distance_cm)
            await self._render_screen(driving_screen)

            # Execute the drive (with concurrent sensor sampling if enabled)
            drive_step = _drive_forward_uncalibrated(self.calibration_distance_cm, speed=0.5)
            sensor_samples: Dict[int, List[float]] = {}

            if ir_sensors:
                stop_sampling = asyncio.Event()
                sampling_task = asyncio.create_task(
                    self._sample_sensors(ir_sensors, stop_sampling)
                )
                try:
                    await drive_step.run_step(robot)
                finally:
                    stop_sampling.set()
                    sensor_samples = await sampling_task
            else:
                await drive_step.run_step(robot)

            # Stop motors after drive completes
            await stop().run_step(robot)

            # Record encoder positions AFTER drive
            encoder_after: Dict[int, int] = {}
            for motor in drive_motors:
                encoder_after[motor.port] = motor.get_position()
                self.debug(f"Motor {motor.port} encoder after: {encoder_after[motor.port]}")

            # Phase 3: Measure - user enters measured distance
            self.debug("Distance calibration: measure phase")
            measured_cm = await self.show(
                DistanceMeasureScreen(self.calibration_distance_cm)
            )

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

                self.per_wheel_results.append(PerWheelCalibration(
                    motor_port=motor.port,
                    old_ticks_to_rad=old_calibration.ticks_to_rad,
                    new_ticks_to_rad=new_ticks_to_rad,
                    delta_ticks=delta_ticks
                ))

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
                # Load YAML config for motor name lookup and persistence
                yaml_config: Optional[Dict] = None
                project_root: Optional[Path] = None
                if self.persist_to_yaml:
                    project_root = _find_project_root()
                    if project_root:
                        try:
                            with open(project_root / "raccoon.project.yml", 'r') as f:
                                yaml_config = yaml.safe_load(f)
                        except (yaml.YAMLError, OSError):
                            self.warn("Could not load raccoon.project.yml for persistence")

                # Apply calibration to each motor
                for result in self.per_wheel_results:
                    # Compute EMA baseline for persistence
                    result.ema_baseline = (
                        result.old_ticks_to_rad * self.ema_alpha +
                        result.new_ticks_to_rad * (1 - self.ema_alpha)
                    )

                    # Find motor name for YAML persistence
                    if yaml_config:
                        result.motor_name = _find_motor_name_by_port(yaml_config, result.motor_port)

                    for motor in drive_motors:
                        if motor.port == result.motor_port:
                            old_cal = motor.get_calibration()
                            new_cal = MotorCalibration()
                            new_cal.ff = old_cal.ff
                            new_cal.pid = old_cal.pid
                            # Apply FULL measured value for this run (best accuracy now)
                            new_cal.ticks_to_rad = result.new_ticks_to_rad
                            new_cal.vel_lpf_alpha = old_cal.vel_lpf_alpha
                            motor.set_calibration(new_cal)

                            self.debug(
                                f"Motor {motor.port}: applied {result.new_ticks_to_rad:.6f} "
                                f"(was {result.old_ticks_to_rad:.6f}, "
                                f"EMA baseline: {result.ema_baseline:.6f})"
                            )
                            break

                # Persist EMA baselines to YAML
                if self.persist_to_yaml and project_root:
                    if _update_yaml_calibration(self.per_wheel_results, project_root):
                        self.info("Updated ticks_to_rad baselines in raccoon.project.yml")
                    else:
                        self.warn("Failed to persist calibration to raccoon.project.yml")

                # Mark as calibrated
                global _calibrated
                _calibrated = True

                self.result = DistanceCalibrationResult(
                    requested_distance_cm=self.calibration_distance_cm,
                    measured_distance_cm=measured_cm,
                    scale_factor=avg_scale_factor
                )
                self.debug(f"Distance calibration applied: {len(self.per_wheel_results)} motor(s) calibrated")

                if self.calibrate_light_sensors and ir_sensors:
                    await self._confirm_light_sensors(ir_sensors, sensor_samples)
                return

            # User wants to retry - loop continues


@dsl(tags=["calibration", "distance"])
def calibrate_distance(
    distance_cm: float = 30.0,
    calibrate_light_sensors: bool = False,
    persist_to_yaml: bool = True,
    ema_alpha: float = DEFAULT_EMA_ALPHA,
) -> CalibrateDistance:
    """
    Create a distance calibration step.

    This calibration:
    1. Applies the measured ticks_to_rad for this run (best accuracy now)
    2. Persists an EMA-filtered baseline to YAML (converges over multiple runs)

    The EMA formula: new_baseline = old × α + measured × (1 - α)
    - α=0.7 (default): ~83% of error absorbed after 5 calibrations
    - α=0.9: Slower, ~41% after 5 calibrations (more stable)
    - α=0.5: Faster, ~97% after 5 calibrations (more responsive)

    Args:
        distance_cm: Distance to drive for calibration (default 30cm)
        calibrate_light_sensors: If True, calibrate IR sensors after distance confirmation
        persist_to_yaml: If True, update raccoon.project.yml with EMA baseline
        ema_alpha: EMA coefficient (0.0-1.0, higher = slower convergence)

    Returns:
        CalibrateDistance step instance
    """
    return CalibrateDistance(
        distance_cm,
        calibrate_light_sensors,
        persist_to_yaml,
        ema_alpha,
    )
