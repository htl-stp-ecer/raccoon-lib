from typing import Optional, TYPE_CHECKING

from libstp.hal import AnalogSensor
from libstp.screen.api import RenderScreen, WFLCalibrationResult
from libstp.step import Step

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class CalibrateWaitForLight(Step):
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
            robot: The robot instance (not used directly but required by Step interface)
        """
        screen = RenderScreen([])  # No IR sensors needed for WFL calibration
        result = await screen.calibrate_wfl(self.sensor)

        if result:
            self.calibration_result = result
            self.debug(
                f"WFL calibration complete: off={result.light_off}, "
                f"on={result.light_on}, threshold={result.threshold}"
            )
        else:
            self.warn("WFL calibration was cancelled or failed")


def calibrate_wait_for_light(sensor: AnalogSensor) -> CalibrateWaitForLight:
    """
    Create a wait-for-light calibration step.

    Args:
        sensor: The AnalogSensor to calibrate

    Returns:
        CalibrateWaitForLight step instance
    """
    return CalibrateWaitForLight(sensor)
