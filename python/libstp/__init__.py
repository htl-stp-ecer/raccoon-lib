from __future__ import annotations

import atexit
import signal
from types import FrameType
from typing import Dict

from libstp import hal as _hal
from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import error, info, debug, warn, initialize_logging
from libstp.sensor_ir import IRSensor, IRSensorCalibration
# Core hardware
Motor = _hal.Motor
Servo = _hal.Servo
AnalogSensor = _hal.AnalogSensor
DigitalSensor = _hal.DigitalSensor

from libstp import hal
from libstp import foundation
from libstp.step import *
from libstp.step import __all__ as _step_all
from libstp.robot.api import GenericRobot, RobotDefinitionsProtocol
from libstp.mission.api import Mission, MissionProtocol
from libstp.timing import StepTimingTracker
from libstp.screen.api import RenderScreen
from libstp.drive import Drive, MotionLimits
from libstp.kinematics_mecanum import MecanumKinematics
from libstp.odometry_fused import FusedOdometry
from libstp.foundation import Feedforward, MotorCalibration, PidGains
from libstp.hal import IMU
from libstp.kinematics_differential import DifferentialKinematics

__all__ = [
    # Core hardware
    "Motor",
    "Servo",
    "AnalogSensor",
    "DigitalSensor",
    # Submodules
    "hal",
    "foundation",
    # Robot API
    "GenericRobot",
    "RobotDefinitionsProtocol",
    "MissionProtocol",
    # Mission API
    "Mission",
    # Logging
    "error",
    "info",
    "debug",
    "warn",
    "initialize_logging",
    "ClassNameLogger",
    # Sensors
    "IRSensor",
    "IRSensorCalibration",
    # Timing
    "StepTimingTracker",
    # Screen
    "RenderScreen",
    # Drive & Kinematics
    "Drive",
    "FusedOdometry",
    "MecanumKinematics",
    "DifferentialKinematics",
    "MotionLimits",
    # Calibration
    "Feedforward",
    "MotorCalibration",
    "PidGains",
    "IMU",
    # All step exports
    *_step_all,
]

_PREVIOUS_SIGNAL_HANDLERS: Dict[int, signal.Handlers] = {}
_HOOKS_INSTALLED = False


def _disable_all_motors() -> None:
    disable = getattr(Motor, "disable_all", None)
    if disable is None:
        return

    try:
        info("Disabling all motors")
        disable()
    except Exception:
        error("Failed to disable all motors")
        # Never let shutdown hooks raise to callers
        pass


def _forward_signal(signum: int, frame: FrameType | None) -> None:
    handler = _PREVIOUS_SIGNAL_HANDLERS.get(signum)
    if handler in (None, signal.SIG_IGN):
        return

    if handler is signal.SIG_DFL:
        if signum == getattr(signal, "SIGINT", None):
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)

    handler(signum, frame)


def _install_shutdown_hooks() -> None:
    global _HOOKS_INSTALLED
    if _HOOKS_INSTALLED:
        return

    _HOOKS_INSTALLED = True

    atexit.register(_disable_all_motors)
    print("Registered atexit shutdown hook to disable all motors")

    for sig_name in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue

        try:
            previous = signal.getsignal(sig)
        except Exception:
            debug(f"Failed to get previous handler for signal {sig_name}")
            continue

        _PREVIOUS_SIGNAL_HANDLERS[sig] = previous

        def handler(signum: int, frame: FrameType | None) -> None:
            _disable_all_motors()
            _forward_signal(signum, frame)

        signal.signal(sig, handler)
        print(f"Registered signal handler for {sig_name}")


_install_shutdown_hooks()
initialize_logging()
