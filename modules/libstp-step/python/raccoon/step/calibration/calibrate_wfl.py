"""
Wait-for-light sensor calibration step using the new UI library.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from raccoon.hal import AnalogSensor
from raccoon.step.annotation import dsl_step
from raccoon.ui.screens.wfl import (
    WFLConfirmScreen,
    WFLMeasureScreen,
)
from raccoon.ui.step import UIStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dataclass
class WFLCalibrationResult:
    """Result of wait-for-light sensor calibration."""

    light_off: float
    light_on: float
    threshold: float


@dsl_step(tags=["calibration", "light"])
class CalibrateWaitForLight(UIStep):
    """Calibrate a wait-for-light sensor via interactive measurement.

    Guides the operator through a two-step measurement flow: first cover
    the sensor (dark reading), then expose it to the start lamp (light
    reading). The midpoint threshold is computed and stored so the
    ``wait_for_light_legacy`` step can detect the lamp signal.

    The operator can retry measurements if the values look wrong before
    confirming the calibration.

    Args:
        sensor: The AnalogSensor instance to calibrate.

    Example::

        from raccoon.step.calibration import calibrate_wait_for_light

        calibrate_wait_for_light(robot.defs.wait_for_light_sensor)
    """

    def __init__(self, sensor: AnalogSensor) -> None:
        super().__init__()
        self.sensor = sensor
        self.calibration_result: WFLCalibrationResult | None = None

    def _generate_signature(self) -> str:
        port = getattr(self.sensor, "port", 0)
        return f"CalibrateWaitForLight(port={port})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping wait-for-light calibration, using stored values")
            return

        port = getattr(self.sensor, "port", 0)

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
                if hasattr(self.sensor, "set_threshold"):
                    self.sensor.set_threshold(confirm_result.threshold)

                self.debug(
                    f"WFL calibration complete: off={confirm_result.light_off}, "
                    f"on={confirm_result.light_on}, threshold={confirm_result.threshold}"
                )
                return

            # User wants to retry - loop continues
