from __future__ import annotations
from abc import abstractmethod
import libstp.class_name_logger
from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import initialize_timer
import libstp.step
from libstp.step import Step
__all__: list[str] = ['ClassNameLogger', 'Mission', 'MissionController', 'SetupMission', 'Step', 'abstractmethod', 'initialize_timer']
class Mission(libstp.class_name_logger.ClassNameLogger):
    def run(self, robot):
        ...
    def sequence(self) -> libstp.step.Step:
        ...
class SetupMission(Mission):
    _custom_pre_start_gate: bool
    async def pre_start_gate(self, robot) -> None:
        ...
class MissionController:
    def __init__(self, robot):
        ...
    def execute_missions(self, missions: typing.List[libstp.mission.api.Mission]):
        ...
