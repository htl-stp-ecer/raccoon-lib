from __future__ import annotations
from abc import abstractmethod
import libstp.class_name_logger
from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import initialize_timer
import libstp.step
from libstp.step import Step
__all__: list[str] = ['ClassNameLogger', 'Mission', 'MissionController', 'Step', 'abstractmethod', 'initialize_timer']
class Mission(libstp.class_name_logger.ClassNameLogger):
    def run(self, robot):
        ...
    def sequence(self) -> libstp.step.Step:
        ...
class MissionController:
    def __init__(self, robot):
        ...
    def execute_missions(self, missions: typing.List[libstp.mission.api.Mission]):
        ...
