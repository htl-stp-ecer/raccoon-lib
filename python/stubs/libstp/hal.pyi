"""
Python bindings for libstp-hal
"""
from __future__ import annotations
import libstp.foundation
__all__: list[str] = ['AnalogSensor', 'DigitalSensor', 'IMU', 'Motor', 'Servo']
class AnalogSensor:
    port: int
    def __init__(self, port: int) -> None:
        ...
    def read(self) -> int:
        ...
class DigitalSensor:
    port: int
    def __init__(self, port: int) -> None:
        ...
    def read(self) -> bool:
        ...
class IMU:
    def __init__(self) -> None:
        ...
    def calibrate(self) -> None:
        """
        Calibrate the IMU sensor
        """
    def get_heading(self) -> float:
        """
        Get firmware-computed heading in radians
        """
    def get_linear_acceleration(self) -> tuple[float, float, float]:
        """
        Get gravity-compensated linear acceleration as (x, y, z) in m/s²
        """
    def get_integrated_velocity(self) -> tuple[float, float, float]:
        """
        Get firmware-integrated velocity as (x, y, z) in m/s
        """
    def reset_integrated_velocity(self) -> None:
        """
        Reset the firmware-integrated velocity accumulator
        """
    def read(self) -> tuple:
        """
        Read acceleration, gyroscope, and magnetometer data
        """
class Motor:
    @staticmethod
    def disable_all() -> None:
        ...
    def __init__(self, port: int, inverted: bool = False, calibration: libstp.foundation.MotorCalibration = ...) -> None:
        ...
    def brake(self) -> None:
        ...
    def off(self) -> None:
        """Disable motor completely (no power, no brake — free-spinning)"""
        ...
    def reset_position_counter(self) -> None:
        """Reset the position counter to zero"""
        ...
    def get_calibration(self) -> libstp.foundation.MotorCalibration:
        ...
    def get_position(self) -> int:
        ...
    def is_done(self) -> bool:
        ...
    def move_relative(self, velocity: int, delta_position: int) -> None:
        ...
    def move_to_position(self, velocity: int, goal_position: int) -> None:
        ...
    def set_speed(self, percent: int) -> None:
        ...
    def set_velocity(self, velocity: int) -> None:
        ...
    @property
    def inverted(self) -> bool:
        ...
    @property
    def port(self) -> int:
        ...
class Servo:
    port: int
    @staticmethod
    def fully_disable_all() -> None:
        ...
    def __init__(self, port: int) -> None:
        ...
    def disable(self) -> None:
        ...
    def enable(self) -> None:
        ...
    def get_position(self) -> float:
        ...
    def set_position(self, position: float) -> None:
        ...
