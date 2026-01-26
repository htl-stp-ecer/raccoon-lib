from libstp.robot.api import GenericRobot
from libstp.screen.api import RenderScreen
from libstp.sensor_ir import IRSensor
from libstp.step import Step


class CalibrateSensors(Step):
    def __init__(self, calibration_time: float = 5.0) -> None:
        super().__init__()
        self.calibration_time = calibration_time

    async def _execute_step(self, robot: "GenericRobot") -> None:
        sensors = robot.defs.analog_sensors
        ir_sensors = [sensor for sensor in sensors if isinstance(sensor, IRSensor)]

        screen = RenderScreen(ir_sensors)
        await screen.calibrate_black_white()


def calibrate_sensors(calibration_time: float = 5.0) -> CalibrateSensors:
    return CalibrateSensors(calibration_time)
