from typing import List

from libstp import calibration_store as CalibrationStore
from libstp import dsl, UIStep, IRSensor
from libstp.calibration_store import CalibrationType


@dsl(hidden=True)
class SwitchCalibrationSet(UIStep):
    """Step that loads a named calibration set and applies it to all IR sensors."""

    def __init__(self, set_name: str = "default") -> None:
        super().__init__()
        self.set_name = set_name

    def _generate_signature(self) -> str:
        return f"SwitchCalibrationSet(set_name={self.set_name!r})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if not CalibrationStore.has_readings(CalibrationType.IR_SENSOR, self.set_name):
            self.warn(f"No calibration data for set '{self.set_name}'")
            return

        values = CalibrationStore.get_readings(CalibrationType.IR_SENSOR, self.set_name)
        white_thresh, black_thresh = values[0], values[1]

        sensors = robot.defs.analog_sensors
        ir_sensors: List[IRSensor] = [s for s in sensors if isinstance(s, IRSensor)]

        for sensor in ir_sensors:
            sensor.setCalibration(black_thresh, white_thresh)

        self.debug(
            f"Applied calibration set '{self.set_name}': "
            f"black={black_thresh}, white={white_thresh}"
        )


@dsl(tags=["calibration", "sensor"])
def switch_calibration_set(set_name: str = "default") -> SwitchCalibrationSet:
    """
    Switch IR sensors to a named calibration set.

    Args:
        set_name: Name of the calibration set to apply (e.g. "default", "transparent")

    Returns:
        SwitchCalibrationSet step instance
    """
    return SwitchCalibrationSet(set_name=set_name)
