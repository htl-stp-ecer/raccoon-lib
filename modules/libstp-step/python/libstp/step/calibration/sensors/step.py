import asyncio
from typing import Optional, List

from libstp import calibration_store as CalibrationStore
from libstp.calibration_store import CalibrationType
from libstp.sensor_ir import IRSensor, IRSensorCalibration
from libstp.step.annotation import dsl
from libstp.ui.step import UIStep

from .dataclasses import IRSensorCalibrationResult, SensorCalibrationData
from .ir_calibrating_screen import IRCalibratingScreen
from .ir_overview_screen import IROverviewScreen
from .ir_results_screen import IRResultsDashboardScreen


@dsl(hidden=True)
class CalibrateSensors(UIStep):
    """Step for calibrating IR sensors (black/white thresholds)."""

    def __init__(
        self,
        calibration_time: float = 5.0,
        allow_use_existing: bool = True,
        calibration_sets: Optional[List[str]] = None,
    ) -> None:
        """
        Initialize the IR sensor calibration step.

        Args:
            calibration_time: Duration for calibration sampling (seconds)
            allow_use_existing: If True, offer existing calibration values
            calibration_sets: Named calibration sets to calibrate. Defaults to ["default"].
        """
        super().__init__()
        self.calibration_time = calibration_time
        self.allow_use_existing = allow_use_existing
        self.calibration_sets = calibration_sets or ["default"]
        self.calibration_result: Optional[IRSensorCalibrationResult] = None

    def _generate_signature(self) -> str:
        return (
            f"CalibrateSensors(time={self.calibration_time}, "
            f"allow_existing={self.allow_use_existing}, "
            f"sets={self.calibration_sets})"
        )

    async def _calibrate_sensors(
            self,
            ir_sensors: List[IRSensor],
            set_name: str = "default",
    ) -> bool:
        """Run the blocking calibration in a thread."""
        return await asyncio.to_thread(
            IRSensorCalibration.calibrateSensors,
            ir_sensors,
            self.calibration_time,
            False,
            set_name,
        )

    def _collect_sensor_data(self, ir_sensors: List[IRSensor], ports: List[int]) -> List[SensorCalibrationData]:
        """Collect per-sensor calibration data after calibration run."""
        sensor_data = []
        for sensor, port in zip(ir_sensors, ports):
            samples = []
            try:
                if hasattr(sensor, '_calibration_values'):
                    samples = [float(v) for v in sensor._calibration_values]
            except Exception:
                pass

            try:
                black_threshold = float(sensor.blackThreshold)
                white_threshold = float(sensor.whiteThreshold)
            except Exception:
                black_threshold = 0.0
                white_threshold = 0.0

            data = SensorCalibrationData(
                port=port,
                samples=samples,
                black_threshold=black_threshold,
                white_threshold=white_threshold,
                black_mean=float(getattr(sensor, 'blackMean', 0.0)),
                white_mean=float(getattr(sensor, 'whiteMean', 0.0)),
                black_std=float(getattr(sensor, 'blackStdDev', 0.0)),
                white_std=float(getattr(sensor, 'whiteStdDev', 0.0)),
            )
            sensor_data.append(data)
        return sensor_data

    async def _calibrate_single_set(
        self,
        robot: "GenericRobot",
        ir_sensors: List[IRSensor],
        ports: List[int],
        set_name: str,
    ) -> bool:
        """Run the calibration flow for a single named set. Returns True if completed."""
        has_existing = all(
            CalibrationStore.has_readings(CalibrationType.IR_SENSOR, f"{set_name}_port{s.port}")
            for s in ir_sensors
        )

        while True:
            show_existing = self.allow_use_existing and has_existing
            choice = await self.show(IROverviewScreen(
                has_existing=show_existing,
                set_name=set_name,
            ))
            await self.close_ui()

            if choice is None:
                self.warn("Calibration UI dismissed before selection; aborting")
                return False

            if choice.use_existing:
                if self.allow_use_existing:
                    self.debug(f"Using existing calibration values for set '{set_name}'")
                    return True
                self.warn("Use existing calibration disabled; recalibrating")

            try:
                calibration_success = await self.run_with_ui(
                    IRCalibratingScreen(ports=ports),
                    self._calibrate_sensors(ir_sensors, set_name),
                )
            except Exception as e:
                self.error(f"Calibration failed: {e}")
                calibration_success = False

            if not calibration_success:
                self.warn("Calibration failed - sensor contrast too low")

            # Collect per-sensor data for the dashboard
            sensor_data = self._collect_sensor_data(ir_sensors, ports)

            dashboard_result = await self.show(
                IRResultsDashboardScreen(sensors=sensor_data)
            )
            await self.close_ui()

            if dashboard_result is None:
                self.warn("Calibration dashboard dismissed; aborting")
                return False

            if dashboard_result.confirmed:
                # Apply per-sensor thresholds
                for sensor, data in zip(ir_sensors, sensor_data):
                    sensor.setCalibration(data.black_threshold, data.white_threshold)
                    self.debug(
                        f"IR calibration for set '{set_name}' port {data.port}: "
                        f"black={data.black_threshold}, white={data.white_threshold}"
                    )

                self.calibration_result = IRSensorCalibrationResult(
                    white_threshold=sensor_data[0].white_threshold,
                    black_threshold=sensor_data[0].black_threshold,
                )
                return True

    async def _execute_step(self, robot: "GenericRobot") -> None:
        sensors = robot.defs.analog_sensors
        ir_sensors: List[IRSensor] = [s for s in sensors if isinstance(s, IRSensor)]

        if not ir_sensors:
            self.warn("No IR sensors found in robot.defs.analog_sensors")
            return

        ports = [s.port for s in ir_sensors]

        for set_name in self.calibration_sets:
            if len(self.calibration_sets) > 1:
                label = set_name.upper()
                proceed = await self.confirm(
                    f"Place sensors on {label} surface, then confirm.",
                    title=f"Calibrate: {label}",
                    yes_label="Ready",
                    no_label="Skip",
                )
                if not proceed:
                    self.debug(f"Skipping calibration set '{set_name}'")
                    continue

            completed = await self._calibrate_single_set(robot, ir_sensors, ports, set_name)
            if not completed:
                return


@dsl(tags=["calibration", "sensor"])
def calibrate_sensors(
    calibration_time: float = 5.0,
    allow_use_existing: bool = True,
    calibration_sets: Optional[List[str]] = None,
) -> CalibrateSensors:
    """
    Create an IR sensor calibration step.

    Args:
        calibration_time: Duration for calibration sampling (seconds)
        allow_use_existing: If True, offer existing calibration values
        calibration_sets: Named calibration sets to calibrate (e.g. ["default", "transparent"])

    Returns:
        CalibrateSensors step instance
    """
    return CalibrateSensors(
        calibration_time,
        allow_use_existing=allow_use_existing,
        calibration_sets=calibration_sets,
    )
