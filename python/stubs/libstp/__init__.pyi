from __future__ import annotations
import _signal
import atexit as atexit
from builtins import frame as FrameType
from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import debug
from libstp.foundation import error
from libstp.foundation import info
from libstp.foundation import initialize_logging
from libstp.hal import Motor
from libstp.hal import Servo
from libstp.mission.api import Mission
from libstp.robot.api import GenericRobot
from libstp.robot.api import MissionProtocol
from libstp.robot.api import RobotDefinitionsProtocol
from libstp.step import SimulationStep
from libstp.step import SimulationStepDelta
from libstp.step import Step
from libstp.step import StepProtocol
import signal as signal
import typing
from . import class_name_logger
from . import foundation
from . import hal
from . import mission
from . import robot
from . import step
__all__: list = ['Motor', 'Servo', 'hal', 'foundation', 'Step', 'StepProtocol', 'SimulationStep', 'SimulationStepDelta', 'GenericRobot', 'RobotDefinitionsProtocol', 'MissionProtocol', 'Mission', 'error', 'info', 'debug', 'initialize_logging', 'ClassNameLogger']
def _disable_all_motors() -> None:
    ...
def _forward_signal(signum: int, frame: FrameType | None) -> None:
    ...
def _install_shutdown_hooks() -> None:
    ...
_HOOKS_INSTALLED: bool = True
_PREVIOUS_SIGNAL_HANDLERS: dict  # value = {<Signals.SIGINT: 2>: _signal.default_int_handler, <Signals.SIGTERM: 15>: <Handlers.SIG_DFL: 0>}
_hal = hal
