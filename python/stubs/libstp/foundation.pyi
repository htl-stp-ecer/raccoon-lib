"""
Python bindings for libstp-foundation
"""
from __future__ import annotations
import typing
__all__: list[str] = ['ChassisVelocity', 'Feedforward', 'MotorCalibration', 'PIDController', 'PidGains', 'Pose', 'debug', 'error', 'info', 'initialize_logging', 'initialize_timer', 'warn']
class ChassisVelocity:
    vx: float
    vy: float
    wz: float
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: float, arg1: float, arg2: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
class Feedforward:
    kA: float
    kS: float
    kV: float
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, kS: float, kV: float, kA: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
class MotorCalibration:
    ff: Feedforward
    pid: PidGains
    ticks_to_rad: float
    vel_lpf_alpha: float
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, ff: Feedforward, pid: PidGains, ticks_to_rad: float, vel_lpf_alpha: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
class PIDController:
    """
    
                A PID controller for managing control loops.
    
                This class provides methods for tuning and executing a PID control loop
                with configurable proportional, integral, and derivative gains.
            
    """
    def __init__(self, Kp: float, Ki: float, Kd: float) -> None:
        """
                             Initialize a PIDController.
        
                             Args:
                                 Kp (float): Proportional gain.
                                 Ki (float): Integral gain.
                                 Kd (float): Derivative gain.
        """
    def calculate(self, error: float) -> float:
        """
                             Calculate the PID output for a given error.
        
                             Args:
                                 error (float): The current error in the system.
        
                             Returns:
                                 float: The calculated PID output, clamped to the specified range.
        """
class PidGains:
    kd: float
    ki: float
    kp: float
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
class Pose:
    position: numpy.ndarray[numpy.float32[3, 1]]
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    @property
    def heading(self) -> float:
        ...
    @heading.setter
    def heading(self, arg1: float) -> None:
        ...
def debug(message: str) -> None:
    """
            Log a message with severity level debug
    """
def error(message: str) -> None:
    """
            Log a message with severity level error
    """
def info(message: str) -> None:
    """
            Log a message with severity level info
    """
def initialize_logging() -> None:
    """
    Initialize and enable the logging system
    """
def initialize_timer() -> None:
    """
    Initialize the timer for elapsed time logging
    """
def warn(message: str) -> None:
    """
            Log a message with severity level warn
    """
