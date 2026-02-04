import asyncio
from typing import Optional, List

from libstp import calibration_store as CalibrationStore
from libstp import dsl, UIStep, IRSensor, IRSensorCalibration
from libstp.calibration_store import CalibrationType

from .dataclasses import IRSensorCalibrationResult
from .ir_calibrating_screen import IRCalibratingScreen
from .ir_confirm_screen import IRConfirmScreen
from .ir_overview_screen import IROverviewScreen


@dsl(hidden=True)
class CalibrateSensors(UIStep):
    """Step for calibrating IR sensors (black/white thresholds)."""

    def __init__(self, calibration_time: float = 5.0, allow_use_existing: bool = True) -> None:
        """
        Initialize the IR sensor calibration step.

        Args:
            calibration_time: Duration for calibration sampling (seconds)
            allow_use_existing: If True, offer existing calibration values
        """
        super().__init__()
        self.calibration_time = calibration_time
        self.allow_use_existing = allow_use_existing
        self.calibration_result: Optional[IRSensorCalibrationResult] = None

    def _generate_signature(self) -> str:
        return (
            f"CalibrateSensors(time={self.calibration_time}, "
            f"allow_existing={self.allow_use_existing})"
        )

    async def _calibrate_sensors(
            self,
            ir_sensors: List[IRSensor],
    ) -> bool:
        """Run the blocking calibration in a thread."""
        return await asyncio.to_thread(
            IRSensorCalibration.calibrateSensors,
            ir_sensors,
            self.calibration_time,
            False,
            # usePre=False
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        sensors = robot.defs.analog_sensors
        ir_sensors: List[IRSensor] = [s for s in sensors if isinstance(s, IRSensor)]

        if not ir_sensors:
            self.warn("No IR sensors found in robot.defs.analog_sensors")
            return

        ports = [s.port for s in ir_sensors]
        has_existing = CalibrationStore.has_readings(CalibrationType.IR_SENSOR)

        while True:
            show_existing = self.allow_use_existing and has_existing
            choice = await self.show(IROverviewScreen(has_existing=show_existing))
            # Ensure the previous screen is removed from the navigation stack
            # before we push the next one. Otherwise older calibration screens
            # remain underneath and reappear after a pop/close.
            await self.close_ui()

            # If the UI was popped (e.g. back gesture or router state restore
            # dropped the screen extra), `show` returns None. In that case we
            # should abort instead of falling through and re‑opening the flow,
            # which is what was causing the calibration screen to come back.
            if choice is None:
                self.warn("Calibration UI dismissed before selection; aborting")
                return

            if choice.use_existing:
                if self.allow_use_existing:
                    self.debug("Using existing calibration values")
                    # TODO: Load from calibration store
                    return
                self.warn("Use existing calibration disabled; recalibrating")

            try:
                calibration_success = await self.run_with_ui(
                    IRCalibratingScreen(ports=ports),
                    self._calibrate_sensors(ir_sensors),
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
            # Pop the confirmation screen so it doesn't sit under the next
            # navigation level (or main app route) after the step completes.
            await self.close_ui()

            # Same guard as above – if the confirmation screen is popped by
            # navigation/state restoration we bail out to avoid reopening the
            # calibration flow unintentionally.
            if confirm_result is None:
                self.warn("Calibration confirmation dismissed; aborting")
                return

            if confirm_result.confirmed:
                for sensor in ir_sensors:
                    sensor.setCalibration(confirm_result.black_threshold, confirm_result.white_threshold)

                self.calibration_result = IRSensorCalibrationResult(
                    white_threshold=confirm_result.white_threshold,
                    black_threshold=confirm_result.black_threshold,
                )

                self.debug(
                    f"IR calibration complete: black={confirm_result.black_threshold}, "
                    f"white={confirm_result.white_threshold}"
                )
                return


@dsl(tags=["calibration", "sensor"])
def calibrate_sensors(
    calibration_time: float = 5.0,
    allow_use_existing: bool = True,
) -> CalibrateSensors:
    """
    Create an IR sensor calibration step.

    Args:
        calibration_time: Duration for calibration sampling (seconds)
        allow_use_existing: If True, offer existing calibration values

    Returns:
        CalibrateSensors step instance
    """
    return CalibrateSensors(calibration_time, allow_use_existing=allow_use_existing)
