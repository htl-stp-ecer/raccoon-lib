"""
Python bindings for libstp-sensors-ir
"""
from __future__ import annotations
import libstp.hal
__all__: list[str] = ['IRSensor', 'IRSensorCalibration']
class IRSensor(libstp.hal.AnalogSensor):
    def __init__(self, port: int, calibrationFactor: float = 1.0) -> None:
        ...
    def probabilityOfBlack(self) -> float:
        ...
    def probabilityOfWhite(self) -> float:
        ...
    def setCalibration(self, arg0: float, arg1: float) -> None:
        ...
class IRSensorCalibration:
    def __init__(self, buttonPort: int = 10) -> None:
        ...
    def calibrateSensors(self, arg0: list[IRSensor], arg1: float) -> bool:
        ...
