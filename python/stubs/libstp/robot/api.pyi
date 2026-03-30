from __future__ import annotations
import abc
from abc import ABC
from abc import abstractmethod
import asyncio as asyncio
import libstp.class_name_logger
from libstp.class_name_logger import ClassNameLogger
import typing
from typing import Any
from typing import Protocol
from typing import runtime_checkable
__all__: list[str] = ['ABC', 'Any', 'ClassNameLogger', 'GenericRobot', 'MissionProtocol', 'Protocol', 'RobotDefinitionsProtocol', 'abstractmethod', 'asyncio', 'runtime_checkable']
class GenericRobot(abc.ABC, libstp.class_name_logger.ClassNameLogger):
    """
    
        Abstract base class for all robots.
    
        Subclasses must implement:
            - defs: Hardware definitions (motors, servos, etc.)
            - drive: Drive system for chassis control
            - odometry: Odometry system for position tracking
    
        Optional attributes:
            - missions: List of missions to execute
            - setup_mission: Mission to run before main missions
            - shutdown_mission: Mission to run after all missions complete
        
    """
    __abstractmethods__: typing.ClassVar[frozenset]  # value = frozenset({'odometry', 'defs', 'drive', 'shutdown_in'})
    _abc_impl: typing.ClassVar[_abc._abc_data]  # value = <_abc._abc_data object>
    def __init__(self) -> None:
        """
        Initialize the robot and log configuration status.
        """
    def _pre_start_gate(self) -> None:
        """
        Wait for light or button press before starting main missions.
        """
    def _run_main_missions(self, missions) -> None:
        """
        Execute main missions sequentially.
        """
    def _run_missions(self) -> None:
        """
        Internal mission execution loop.
        """
    def start(self) -> None:
        """
        
                Start executing the robot's missions.
        
                Runs setup_mission (if present), then all missions in order,
                then shutdown_mission (if present).
        
                Note: This method blocks until all missions complete.
                For non-blocking execution, use start_async() instead.
                
        """
    def start_async(self) -> None:
        """
        
                Async version of start() for use in existing event loops.
        
                Useful for testing or integration with other async code.
                
        """
    @property
    def defs(self) -> RobotDefinitionsProtocol:
        """
        Hardware definitions (motors, servos, sensors).
        """
    @property
    def drive(self) -> Drive:
        """
        Drive system for chassis velocity control.
        """
    @property
    def missions(self) -> typing.List[libstp.robot.api.MissionProtocol]:
        """
        List of missions to execute. Override to provide missions.
        """
    @property
    def odometry(self) -> Odometry:
        """
        Odometry system for position tracking.
        """
    @property
    def setup_mission(self) -> typing.Optional[libstp.mission.api.SetupMission]:
        """
        Optional setup mission to run before main missions.
        Must be a SetupMission instance.
        """
    @property
    def shutdown_mission(self) -> typing.Optional[libstp.robot.api.MissionProtocol]:
        """
        Optional mission to run after all missions complete.
        """
    @property
    def shutdown_in(self) -> float:
        """
        Maximum runtime in seconds for main missions.
        """
class MissionProtocol(typing.Protocol):
    """
    Protocol for mission objects that can be run on a robot.
    """
    __abstractmethods__: typing.ClassVar[frozenset]  # value = frozenset()
    __parameters__: typing.ClassVar[tuple] = tuple()
    _abc_impl: typing.ClassVar[_abc._abc_data]  # value = <_abc._abc_data object>
    _is_protocol: typing.ClassVar[bool] = True
    _is_runtime_protocol: typing.ClassVar[bool] = True
    @staticmethod
    def __subclasshook__(other):
        ...
    def __init__(self, *args, **kwargs):
        ...
    def run(self, robot: GenericRobot) -> None:
        """
        Execute the mission on the given robot.
        """
class RobotDefinitionsProtocol(typing.Protocol):
    """
    
        Protocol defining the structure of robot hardware definitions.
    
        Implementations should define motor and servo attributes as class variables.
        Example:
            class MyDefs:
                left_motor = Motor(port=0, ...)
                right_motor = Motor(port=1, ...)
                arm_servo = Servo(port=0, ...)
        
    """
    __abstractmethods__: typing.ClassVar[frozenset]  # value = frozenset()
    __parameters__: typing.ClassVar[tuple] = tuple()
    _abc_impl: typing.ClassVar[_abc._abc_data]  # value = <_abc._abc_data object>
    _is_protocol: typing.ClassVar[bool] = True
    _is_runtime_protocol: typing.ClassVar[bool] = True
    @staticmethod
    def __subclasshook__(other):
        ...
    def __init__(self, *args, **kwargs):
        ...
