from typing import List

from raccoon import calibration_store as CalibrationStore
from raccoon.calibration_store import CalibrationType
from raccoon.sensor_ir import IRSensor
from raccoon.step.annotation import dsl_step
from raccoon.ui.step import UIStep


@dsl_step(tags=["calibration", "sensor"])
class SwitchCalibrationSet(UIStep):
    """Switch IR sensors to a named calibration set.

    Loads calibration data for the given set name from the calibration
    store and applies it to all registered IR sensors. Each sensor looks
    up its per-port calibration key (e.g. ``"transparent_port3"``) and
    sets its black/white thresholds accordingly.

    Use this to swap between surface-specific calibrations at runtime
    (e.g. switching from the default table surface to transparent objects).

    Args:
        set_name: Name of the calibration set to apply
            (e.g. ``"default"``, ``"transparent"``).

    Example::

        from raccoon.step.calibration import switch_calibration_set

        # Switch to transparent calibration before scoring
        switch_calibration_set("transparent")
    """

    def __init__(self, set_name: str = "default") -> None:
        super().__init__()
        self.set_name = set_name

    def _generate_signature(self) -> str:
        return f"SwitchCalibrationSet(set_name={self.set_name!r})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        sensors = robot.defs.analog_sensors
        ir_sensors: List[IRSensor] = [s for s in sensors if isinstance(s, IRSensor)]

        for sensor in ir_sensors:
            key = f"{self.set_name}_port{sensor.port}"
            if not CalibrationStore.has_readings(CalibrationType.IR_SENSOR, key):
                self.warn(f"No calibration data for set '{key}'")
                continue

            values = CalibrationStore.get_readings(CalibrationType.IR_SENSOR, key)
            white_thresh, black_thresh = values[0], values[1]
            sensor.setCalibration(black_thresh, white_thresh)

            self.debug(
                f"Applied calibration set '{key}': "
                f"black={black_thresh}, white={white_thresh}"
            )
