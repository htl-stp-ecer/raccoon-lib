from __future__ import annotations

import atexit
import signal
from types import FrameType
from typing import Dict

from libstp._core import __version__ as __version__

try:
    from raccoon_transport import __version__ as _raccoon_version
except Exception:
    _raccoon_version = "unknown"

from libstp.no_calibrate import is_no_calibrate as _is_no_calibrate

_no_cal_label = " | --no-calibrate" if _is_no_calibrate() else ""
print(f"libstp v{__version__} | raccoon-transport v{_raccoon_version}{_no_cal_label}")

from libstp import hal as _hal
from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import initialize_logging
from libstp.log import error, info, debug, warn, trace
from libstp.sensor_ir import IRSensor, IRSensorCalibration
from libstp.sensor_et import ETSensor
from libstp.cam import CamSensor as CamSensor
# Core hardware
Motor = _hal.Motor
Servo = _hal.Servo
AnalogSensor = _hal.AnalogSensor
DigitalSensor = _hal.DigitalSensor

from libstp import hal
from libstp import foundation
from libstp.step import *
from libstp.step import __all__ as _step_all
from libstp.step.servo.preset import ServoPreset
from libstp.ui import __all__ as _ui_all
from libstp.robot import __all__ as _robot_all
from libstp.robot import *
from libstp.mission.api import Mission, MissionProtocol
from libstp.timing import StepTimingTracker
# UI library (replaces legacy RenderScreen)
from libstp.ui import *
from libstp.drive import Drive, AxisVelocityControlConfig, ChassisVelocityControlConfig
from libstp.motion import UnifiedMotionPidConfig
from libstp.kinematics_mecanum import MecanumKinematics
from libstp.odometry_fused import FusedOdometry, FusedOdometryConfig
from libstp.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig
from libstp.foundation import Feedforward, FeedforwardController, MotorCalibration, PidConfig, PidController, PidGains
from libstp.motion import AxisConstraints
from libstp.hal import IMU
from libstp.kinematics_differential import DifferentialKinematics
from libstp.kmeans import KMeans, KMeansResult

__all__ = [
    # Core hardware
    "Motor",
    "Servo",
    "ServoPreset",
    "AnalogSensor",
    "DigitalSensor",
    # Submodules
    "hal",
    "foundation",
    # Robot API
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
    "ETSensor",
    "CamSensor",
    # Timing
    "StepTimingTracker",
    # UI
    "UIStep",
    "UIScreen",
    # Drive & Kinematics
    "Drive",
    "FusedOdometry",
    "FusedOdometryConfig",
    "Stm32Odometry",
    "Stm32OdometryConfig",
    "MecanumKinematics",
    "DifferentialKinematics",
    "AxisVelocityControlConfig",
    "ChassisVelocityControlConfig",
    "UnifiedMotionPidConfig",
    "KMeans",
    "KMeansResult",
    # Calibration
    "Feedforward",
    "FeedforwardController",
    "MotorCalibration",
    "PidConfig",
    "PidController",
    "PidGains",
    "IMU",
    # Motion
    "AxisConstraints",
    # All step exports
    *_robot_all,
    *_step_all,
    *_ui_all,
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


def _shutdown_logging() -> None:
    """Tear down spdlog before C++ static destruction begins."""
    try:
        foundation.shutdown_logging()
    except Exception:
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

    atexit.register(_shutdown_logging)
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
