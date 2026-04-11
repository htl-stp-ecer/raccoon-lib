from __future__ import annotations

import atexit
import signal
from types import FrameType
from typing import Dict

from raccoon._core import __version__ as __version__

try:
    from raccoon_transport import __version__ as _raccoon_version
except Exception:
    _raccoon_version = "unknown"

from raccoon.no_calibrate import is_no_calibrate as _is_no_calibrate
from raccoon.no_checkpoints import is_no_checkpoints as _is_no_checkpoints

_no_cal_label = " | --no-calibrate" if _is_no_calibrate() else ""
_no_chk_label = " | --no-checkpoints" if _is_no_checkpoints() else ""
print(f"raccoon v{__version__} | raccoon-transport v{_raccoon_version}{_no_cal_label}{_no_chk_label}")

from raccoon import hal as _hal
from raccoon.class_name_logger import ClassNameLogger
from raccoon.foundation import initialize_logging
from raccoon.log import error, info, debug, warn, trace
from raccoon.sensor_ir import IRSensor, IRSensorCalibration
from raccoon.sensor_et import ETSensor
from raccoon.cam import CamSensor as CamSensor
# Core hardware
Motor = _hal.Motor
Servo = _hal.Servo
AnalogSensor = _hal.AnalogSensor
DigitalSensor = _hal.DigitalSensor

from raccoon import hal
from raccoon import foundation
from raccoon.step import *
from raccoon.step import __all__ as _step_all
from raccoon.step.servo.preset import ServoPreset
from raccoon.ui import __all__ as _ui_all
from raccoon.robot import __all__ as _robot_all
from raccoon.robot import *
from raccoon.mission.api import Mission, MissionProtocol, SetupMission
from raccoon.timing import StepTimingTracker
# UI library (replaces legacy RenderScreen)
from raccoon.ui import *
from raccoon.drive import Drive, AxisVelocityControlConfig, ChassisVelocityControlConfig
from raccoon.motion import UnifiedMotionPidConfig
from raccoon.kinematics_mecanum import MecanumKinematics
from raccoon.odometry_fused import FusedOdometry, FusedOdometryConfig
from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig
from raccoon.foundation import Feedforward, FeedforwardController, MotorCalibration, PidConfig, PidController, PidGains
from raccoon.motion import AxisConstraints
from raccoon.hal import IMU
from raccoon.hal import IOdometryBridge, OdometryBridge
from raccoon.kinematics_differential import DifferentialKinematics
from raccoon.kmeans import KMeans, KMeansResult
from raccoon.robot.table_map import TableMap, MapSegment
from raccoon.robot.map_corrected_odometry import MapCorrectedOdometry

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
    "SetupMission",
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
    # Table map
    "TableMap",
    "MapSegment",
    "MapCorrectedOdometry",
    # Calibration
    "Feedforward",
    "FeedforwardController",
    "MotorCalibration",
    "PidConfig",
    "PidController",
    "PidGains",
    "IMU",
    "IOdometryBridge",
    "OdometryBridge",
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
