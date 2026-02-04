"""
Wait-for-light sensor calibration step using the new UI library.
"""

from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

from libstp.hal import AnalogSensor
from libstp.step.annotation import dsl
from libstp.ui.step import UIStep
from libstp.ui.screens.wfl import (
    WFLMeasureScreen,
    WFLConfirmScreen,
    WFLConfirmResult,
)

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dataclass
class WFLCalibrationResult:
    """Result of wait-for-light sensor calibration."""
    light_off: float
    light_on: float
    threshold: float


@dsl(hidden=True)
class CalibrateWaitForLight(UIStep):
    """Step for calibrating a wait-for-light sensor."""

    def __init__(self, sensor: AnalogSensor) -> None:
        """
        Initialize the wait-for-light calibration step.

        Args:
            sensor: The AnalogSensor instance to calibrate
        """
        super().__init__()
        self.sensor = sensor
        self.calibration_result: Optional[WFLCalibrationResult] = None

    def _generate_signature(self) -> str:
        port = getattr(self.sensor, 'port', 0)
        return f"CalibrateWaitForLight(port={port})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Execute the wait-for-light calibration.

        The calibration flow:
        1. User covers sensor (dark reading)
        2. User exposes sensor to light (bright reading)
        3. User confirms the calibration values
        4. Threshold is computed and stored

        Args:
            robot: The robot instance
        """
        port = getattr(self.sensor, 'port', 0)

        while True:
            # Step 1: Measure light OFF (covered)
            off_result = await self.show(WFLMeasureScreen(port=port, is_on=False))
            light_off = off_result.value

            # Step 2: Measure light ON (exposed)
            on_result = await self.show(WFLMeasureScreen(port=port, is_on=True))
            light_on = on_result.value

            # Step 3: Confirm values
            confirm_result = await self.show(
                WFLConfirmScreen(port=port, light_off=light_off, light_on=light_on)
            )

            if confirm_result.confirmed:
                # Store result
                self.calibration_result = WFLCalibrationResult(
                    light_off=confirm_result.light_off,
                    light_on=confirm_result.light_on,
                    threshold=confirm_result.threshold,
                )

                # Apply to sensor if it has a set_threshold method
                if hasattr(self.sensor, 'set_threshold'):
                    self.sensor.set_threshold(confirm_result.threshold)

                self.debug(
                    f"WFL calibration complete: off={confirm_result.light_off}, "
                    f"on={confirm_result.light_on}, threshold={confirm_result.threshold}"
                )
                return

            # User wants to retry - loop continues


@dsl(tags=["calibration", "light"])
def calibrate_wait_for_light(sensor: AnalogSensor) -> CalibrateWaitForLight:
    """
    Create a wait-for-light calibration step.

    Args:
        sensor: The AnalogSensor to calibrate

    Returns:
        CalibrateWaitForLight step instance
    """
    return CalibrateWaitForLight(sensor)
