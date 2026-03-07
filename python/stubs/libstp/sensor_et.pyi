"""
Python bindings for libstp-sensor-et
"""
from __future__ import annotations
import libstp.hal
__all__: list[str] = ['ETSensor']
class ETSensor(libstp.hal.AnalogSensor):
    def __init__(self, port: int) -> None:
        ...
    def raw(self) -> int:
        ...
