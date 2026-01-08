from __future__ import annotations
from abc import abstractmethod
import dataclasses
from dataclasses import dataclass
import libstp.class_name_logger
from libstp.class_name_logger import ClassNameLogger
import libstp.robot.api
from libstp.robot.api import GenericRobot
import time as time
import typing
from typing import Protocol
from typing import runtime_checkable
__all__: list[str] = ['ClassNameLogger', 'GenericRobot', 'Protocol', 'SimulationStep', 'SimulationStepDelta', 'Step', 'StepProtocol', 'abstractmethod', 'dataclass', 'runtime_checkable', 'time']
class SimulationStep:
    """
    SimulationStep(id: str, label: str | None, average_duration_ms: float, duration_stddev_ms: float, delta: libstp.step.SimulationStepDelta)
    """
    __dataclass_fields__: typing.ClassVar[dict]  # value = {'id': Field(name='id',type=<class 'str'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'label': Field(name='label',type=str | None,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'average_duration_ms': Field(name='average_duration_ms',type=<class 'float'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'duration_stddev_ms': Field(name='duration_stddev_ms',type=<class 'float'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'delta': Field(name='delta',type=<class 'libstp.step.SimulationStepDelta'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD)}
    __dataclass_params__: typing.ClassVar[dataclasses._DataclassParams]  # value = _DataclassParams(init=True,repr=True,eq=True,order=False,unsafe_hash=False,frozen=False)
    __hash__: typing.ClassVar[None] = None
    __match_args__: typing.ClassVar[tuple] = ('id', 'label', 'average_duration_ms', 'duration_stddev_ms', 'delta')
    def __eq__(self, other):
        ...
    def __init__(self, id: str, label: str | None, average_duration_ms: float, duration_stddev_ms: float, delta: SimulationStepDelta) -> None:
        ...
    def __repr__(self):
        ...
class SimulationStepDelta:
    """
    SimulationStepDelta(forward: float, strafe: float, angular: float)
    """
    __dataclass_fields__: typing.ClassVar[dict]  # value = {'forward': Field(name='forward',type=<class 'float'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'strafe': Field(name='strafe',type=<class 'float'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'angular': Field(name='angular',type=<class 'float'>,default=<dataclasses._MISSING_TYPE object>,default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD)}
    __dataclass_params__: typing.ClassVar[dataclasses._DataclassParams]  # value = _DataclassParams(init=True,repr=True,eq=True,order=False,unsafe_hash=False,frozen=False)
    __hash__: typing.ClassVar[None] = None
    __match_args__: typing.ClassVar[tuple] = ('forward', 'strafe', 'angular')
    def __eq__(self, other):
        ...
    def __init__(self, forward: float, strafe: float, angular: float) -> None:
        ...
    def __repr__(self):
        ...
class Step(libstp.class_name_logger.ClassNameLogger):
    def __init__(self) -> None:
        ...
    def _execute_step(self, robot: libstp.robot.api.GenericRobot) -> None:
        """
        Actual step logic implemented by subclasses.
        """
    def _generate_signature(self) -> str:
        """
        
                Generate a unique signature for this step and its parameters.
                Override in subclasses to include configuration details.
                
        """
    def run_step(self, robot: libstp.robot.api.GenericRobot) -> None:
        """
        
                Execute the step logic with execution timing instrumentation.
                
        """
    def to_simulation_step(self) -> SimulationStep:
        """
        
                Convert this step to a SimulationStep representation.
        
                Default implementation uses timing data from the database if available,
                otherwise returns reasonable defaults. Override this method in subclasses
                to provide accurate delta values for position/heading changes.
                
        """
class StepProtocol(typing.Protocol):
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
    def run_step(self, robot: libstp.robot.api.GenericRobot) -> None:
        ...
