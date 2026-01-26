import asyncio
from dataclasses import dataclass
from typing import Dict, List, Optional, TYPE_CHECKING

from libstp.screen.api import RenderScreen, DistanceCalibrationResult, LightSensorCalibrationResult
from libstp.step import Step

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot
    from libstp.sensor_ir import IRSensor


class CalibrationRequiredError(Exception):
    """Raised when an operation requires calibration but none has been performed."""
    pass


# Simple calibration flag (runtime only)
_calibrated: bool = False


def is_distance_calibrated() -> bool:
    """Check if distance calibration has been performed."""
    return _calibrated


def require_distance_calibration() -> None:
    """Raise CalibrationRequiredError if not calibrated."""
    if not _calibrated:
        raise CalibrationRequiredError(
            "Distance calibration required. Run calibrate_distance() first."
        )


def reset_distance_calibration() -> None:
    """Reset the calibration flag (for testing)."""
    global _calibrated
    _calibrated = False


@dataclass
class PerWheelCalibration:
    """Result of per-wheel distance calibration."""
    motor_port: int
    old_ticks_to_rad: float
    new_ticks_to_rad: float
    delta_ticks: int


class CalibrateDistance(Step):
    """Step for calibrating robot distance estimation via per-wheel ticks_to_rad adjustment."""

    def __init__(
        self,
        calibration_distance_cm: float = 30.0,
        calibrate_light_sensors: bool = False
    ) -> None:
        """
        Initialize the distance calibration step.

        Args:
            calibration_distance_cm: Distance to drive for calibration (default 30cm)
            calibrate_light_sensors: If True, also sample IR sensors during drive
                                     and compute calibration thresholds
        """
        super().__init__()
        self.calibration_distance_cm = calibration_distance_cm
        self.calibrate_light_sensors = calibrate_light_sensors
        self.result: Optional[DistanceCalibrationResult] = None
        self.light_sensor_results: List[LightSensorCalibrationResult] = []
        self.per_wheel_results: List[PerWheelCalibration] = []

    def _generate_signature(self) -> str:
        return f"CalibrateDistance(distance_cm={self.calibration_distance_cm}, light_sensors={self.calibrate_light_sensors})"

    async def _sample_sensors(
        self,
        sensors: List["IRSensor"],
        stop_event: asyncio.Event
    ) -> Dict[int, List[float]]:
        """Sample IR sensors at 10ms intervals until stop_event is set."""
        samples: Dict[int, List[float]] = {sensor.port: [] for sensor in sensors}
        while not stop_event.is_set():
            for sensor in sensors:
                samples[sensor.port].append(float(sensor.read()))
            await asyncio.sleep(0.01)  # 10ms interval
        return samples

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Execute the distance calibration.

        The calibration flow:
        1. Show prepare screen - user places robot
        2. Record encoder positions for all drive motors
        3. User presses button - robot drives requested distance
           (optionally sampling IR sensors during drive)
        4. Record encoder positions after drive
        5. Show measure screen - user enters measured distance
        6. Calculate new ticks_to_rad for each motor
        7. Apply calibration to each motor

        Args:
            robot: The robot instance for driving
        """
        from libstp.step.drive import _drive_forward_uncalibrated
        from libstp.step.stop import stop
        from libstp.sensor_ir import IRSensor
        from libstp.foundation import MotorCalibration

        screen = RenderScreen([])
        screen.change_screen("calibrate_sensors")

        # Get wheel radius from kinematics
        wheel_radius = robot.drive.get_wheel_radius()
        self.debug(f"Wheel radius: {wheel_radius:.4f}m")

        # Get drive motors from kinematics (single source of truth)
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
            if not ir_sensors:
                self.warn("No IR sensors found in robot.defs.analog_sensors")
            else:
                self.debug(f"Will calibrate {len(ir_sensors)} IR sensor(s) during drive")

        # Phase 1: Prepare - user places robot
        self.debug("Distance calibration: prepare phase")
        screen.send_state({
            "type": "distanceCalibration",
            "state": "prepare",
            "requested_distance_cm": self.calibration_distance_cm,
            "calibrating_light_sensors": self.calibrate_light_sensors and len(ir_sensors) > 0
        })
        await screen._RenderScreen__wait_for_button()

        # Record encoder positions BEFORE drive
        encoder_before: Dict[int, int] = {}
        for motor in drive_motors:
            encoder_before[motor.port] = motor.get_position()
            self.debug(f"Motor {motor.port} encoder before: {encoder_before[motor.port]}")

        # Phase 2: Driving - robot moves (with optional sensor sampling)
        self.debug(f"Distance calibration: driving {self.calibration_distance_cm}cm")
        screen.send_state({
            "type": "distanceCalibration",
            "state": "driving",
            "requested_distance_cm": self.calibration_distance_cm,
            "calibrating_light_sensors": self.calibrate_light_sensors and len(ir_sensors) > 0
        })

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

        # Calibrate sensors with collected samples
        self.light_sensor_results = []
        if ir_sensors and sensor_samples:
            for sensor in ir_sensors:
                values = sensor_samples.get(sensor.port, [])
                if values:
                    success = sensor.calibrate(values)
                    result = LightSensorCalibrationResult(
                        sensor_port=sensor.port,
                        black_threshold=sensor.blackThreshold if success else 0.0,
                        white_threshold=sensor.whiteThreshold if success else 0.0,
                        num_samples=len(values),
                        success=success
                    )
                    self.light_sensor_results.append(result)
                    if success:
                        self.debug(
                            f"Sensor {sensor.port} calibrated: "
                            f"black={sensor.blackThreshold:.1f}, white={sensor.whiteThreshold:.1f}"
                        )
                    else:
                        self.warn(f"Sensor {sensor.port} calibration failed with {len(values)} samples")

        # Phase 3: Measure - user enters measured distance
        self.debug("Distance calibration: measure phase")
        screen.send_state({
            "type": "distanceCalibration",
            "state": "measure",
            "requested_distance_cm": self.calibration_distance_cm
        })

        # Wait for user to enter measured distance (5 minute timeout)
        response = await screen._RenderScreen__wait_for_lcm_response(timeout=300)

        if response.value == "retry":
            self.warn("Distance calibration timeout during measure phase")
            return

        if response.value == "cancelled":
            self.debug("Distance calibration cancelled")
            return

        # Parse measured distance from response reason
        measured_cm = self._parse_measured_distance(response.reason)
        if measured_cm is None:
            self.warn(f"Could not parse measured distance from: {response.reason}")
            return

        measured_m = measured_cm / 100.0
        self.debug(f"Measured distance: {measured_m:.4f}m")

        # Calculate new ticks_to_rad for each motor
        # Formula: new_ticks_to_rad = (measured_m / wheel_radius) / |delta_ticks|
        # This gives the radians per tick that would produce the measured distance
        self.per_wheel_results = []
        for motor in drive_motors:
            delta_ticks = encoder_after[motor.port] - encoder_before[motor.port]
            old_calibration = motor.get_calibration()

            if abs(delta_ticks) < 10:
                self.warn(f"Motor {motor.port} had insufficient movement ({delta_ticks} ticks), skipping calibration")
                continue

            # Calculate the angle the wheel actually traveled (in radians)
            # arc_length = radius * theta => theta = arc_length / radius
            theta_rad = measured_m / wheel_radius

            # new_ticks_to_rad converts ticks to radians
            new_ticks_to_rad = theta_rad / abs(delta_ticks)

            self.per_wheel_results.append(PerWheelCalibration(
                motor_port=motor.port,
                old_ticks_to_rad=old_calibration.ticks_to_rad,
                new_ticks_to_rad=new_ticks_to_rad,
                delta_ticks=delta_ticks
            ))

            self.debug(
                f"Motor {motor.port}: delta_ticks={delta_ticks}, "
                f"old_ticks_to_rad={old_calibration.ticks_to_rad:.6f}, "
                f"new_ticks_to_rad={new_ticks_to_rad:.6f}"
            )

        if not self.per_wheel_results:
            self.warn("No motors were calibrated due to insufficient movement")
            return

        # Calculate overall scale factor for display (average adjustment)
        scale_factors = [
            r.new_ticks_to_rad / r.old_ticks_to_rad
            for r in self.per_wheel_results
            if r.old_ticks_to_rad > 0
        ]
        avg_scale_factor = sum(scale_factors) / len(scale_factors) if scale_factors else 1.0

        # Phase 4: Confirm - show results
        confirm_state = {
            "type": "distanceCalibration",
            "state": "confirm",
            "requested_distance_cm": self.calibration_distance_cm,
            "measured_distance_cm": measured_cm,
            "scale_factor": avg_scale_factor,
            "per_wheel_results": [
                {
                    "port": r.motor_port,
                    "old_ticks_to_rad": r.old_ticks_to_rad,
                    "new_ticks_to_rad": r.new_ticks_to_rad,
                    "delta_ticks": r.delta_ticks
                }
                for r in self.per_wheel_results
            ]
        }

        # Add light sensor results if any
        if self.light_sensor_results:
            confirm_state["light_sensor_results"] = [
                {
                    "port": r.sensor_port,
                    "black_threshold": r.black_threshold,
                    "white_threshold": r.white_threshold,
                    "num_samples": r.num_samples,
                    "success": r.success
                }
                for r in self.light_sensor_results
            ]

        screen.send_state(confirm_state)

        confirm_response = await screen._RenderScreen__wait_for_lcm_response(timeout=120)

        if confirm_response.value == "confirmed":
            # Apply calibration to each motor
            for result in self.per_wheel_results:
                for motor in drive_motors:
                    if motor.port == result.motor_port:
                        old_cal = motor.get_calibration()
                        # Create new calibration with updated ticks_to_rad
                        new_cal = MotorCalibration()
                        new_cal.ff = old_cal.ff
                        new_cal.pid = old_cal.pid
                        new_cal.ticks_to_rad = result.new_ticks_to_rad
                        new_cal.vel_lpf_alpha = old_cal.vel_lpf_alpha
                        motor.set_calibration(new_cal)
                        self.debug(f"Applied calibration to motor {motor.port}: ticks_to_rad={result.new_ticks_to_rad:.6f}")
                        break

            # Mark as calibrated
            global _calibrated
            _calibrated = True

            self.result = DistanceCalibrationResult(
                requested_distance_cm=self.calibration_distance_cm,
                measured_distance_cm=measured_cm,
                scale_factor=avg_scale_factor
            )
            self.debug(f"Distance calibration applied: {len(self.per_wheel_results)} motor(s) calibrated")

        elif confirm_response.value == "retry":
            # User wants to retry
            self.debug("User requested retry")
            await self._execute_step(robot)
        else:
            self.debug("Distance calibration cancelled at confirm phase")

    def _parse_measured_distance(self, reason: str) -> Optional[float]:
        """Parse measured distance from LCM response reason field."""
        try:
            # Expected format: "measured_distance=28.5"
            if "measured_distance=" in reason:
                value_str = reason.split("measured_distance=")[1].split()[0]
                return float(value_str)
            # Also try just a raw number
            return float(reason)
        except (ValueError, IndexError):
            return None


def calibrate_distance(
    distance_cm: float = 30.0,
    calibrate_light_sensors: bool = False
) -> CalibrateDistance:
    """
    Create a distance calibration step.

    The robot will drive the specified distance, then prompt the user
    to measure the actual distance traveled. Per-wheel ticks_to_rad
    calibration values are computed and applied to each drive motor.

    Args:
        distance_cm: Distance to drive for calibration (default 30cm)
        calibrate_light_sensors: If True, also sample IR sensors during the
                                 drive and compute black/white thresholds

    Returns:
        CalibrateDistance step instance
    """
    return CalibrateDistance(distance_cm, calibrate_light_sensors)
