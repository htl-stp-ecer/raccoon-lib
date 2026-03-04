import asyncio
from typing import Optional, List

from libstp import calibration_store as CalibrationStore
from libstp.calibration_store import CalibrationType
from libstp.sensor_ir import IRSensor, IRSensorCalibration
from libstp.step.annotation import dsl
from libstp.ui.step import UIStep

from .dataclasses import IRSensorCalibrationResult
from .ir_calibrating_screen import IRCalibratingScreen
from .ir_confirm_screen import IRConfirmScreen
from .ir_overview_screen import IROverviewScreen


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

    async def _calibrate_single_set(
        self,
        robot: "GenericRobot",
        ir_sensors: List[IRSensor],
        ports: List[int],
        set_name: str,
    ) -> bool:
        """Run the calibration flow for a single named set. Returns True if completed."""
        has_existing = CalibrationStore.has_readings(CalibrationType.IR_SENSOR, set_name)

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

            black_thresh = ir_sensors[0].blackThreshold
            white_thresh = ir_sensors[0].whiteThreshold

            collected_values = []
            if hasattr(ir_sensors[0], '_calibration_values'):
                collected_values = ir_sensors[0]._calibration_values

            confirm_result = await self.show(
                IRConfirmScreen(
                    black_threshold=black_thresh,
                    white_threshold=white_thresh,
                    collected_values=collected_values,
                )
            )
            await self.close_ui()

            if confirm_result is None:
                self.warn("Calibration confirmation dismissed; aborting")
                return False

            if confirm_result.confirmed:
                for sensor in ir_sensors:
                    sensor.setCalibration(confirm_result.black_threshold, confirm_result.white_threshold)

                self.calibration_result = IRSensorCalibrationResult(
                    white_threshold=confirm_result.white_threshold,
                    black_threshold=confirm_result.black_threshold,
                )

                self.debug(
                    f"IR calibration complete for set '{set_name}': "
                    f"black={confirm_result.black_threshold}, "
                    f"white={confirm_result.white_threshold}"
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
