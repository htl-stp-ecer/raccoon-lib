from __future__ import annotations

import atexit
import contextlib
import signal
from types import FrameType

from raccoon._core import __version__

# Minimum project format_version required by this version of raccoon-lib.
# Bump this when a breaking YAML change lands (e.g. a robot.yml key renamed)
# so that raccoon-cli can warn the user to run `raccoon migrate`.
MIN_FORMAT_VERSION: int = 1

try:
    from raccoon_transport import __version__ as _raccoon_version
except Exception:
    _raccoon_version = "unknown"

from raccoon.no_calibrate import is_no_calibrate as _is_no_calibrate
from raccoon.no_checkpoints import is_no_checkpoints as _is_no_checkpoints

from raccoon import hal as _hal
from raccoon.class_name_logger import ClassNameLogger
from raccoon.foundation import initialize_logging
from raccoon.log import error, info, debug, warn, trace
from raccoon.sensor_ir import IRSensor, IRSensorCalibration
from raccoon.sensor_et import ETSensor

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
from raccoon.step.arm.preset import ArmPreset
from raccoon.ui import __all__ as _ui_all
from raccoon.robot import __all__ as _robot_all
from raccoon.robot import *
from raccoon.mission.api import Mission, MissionProtocol, SetupMission
from raccoon.timing import StepTimingTracker

from raccoon.foundation import get_transport, shutdown_transport


def _check_iceoryx2_shm_access() -> None:
    """Fail fast with a clear message if iceoryx2 SHM is owned by another uid.

    iceoryx2 stores its global management segment as ``/dev/shm/iox2_*_node.global_mgmt``
    and its service discovery state under ``/tmp/iceoryx2/``. Once a process
    (typically one accidentally launched as root during a deploy / debug
    session) creates these as ``root:root mode 700``, every subsequent iox2
    process running as a normal user dies inside Rust with
    ``NodeCreationFailure(InternalError)``. That is an ``abort()`` from
    ``NodeBuilder::create().value()``; no Python ``try/except`` can catch it
    and the only diagnostic on stderr is a cryptic mangled type name. Detect
    the precondition here, before ``get_transport()`` constructs the Node,
    and emit actionable cleanup steps instead.
    """
    import os
    import sys
    from pathlib import Path

    uid = os.getuid()
    blocked: list[str] = []

    # /dev/shm: every iox2_* segment must be ours or world-accessible. The
    # *_node.global_mgmt one is the truly fatal case (all iox2 processes
    # share that single segment), but a foreign-owned per-service segment
    # would fail just as hard on first open(), so we check them all uniformly.
    for path in Path("/dev/shm").glob("iox2_*"):
        try:
            st = path.stat()
        except OSError:
            continue
        if st.st_uid == uid:
            continue
        # World-readable+writable would actually work for iox2, allow it.
        if (st.st_mode & 0o006) == 0o006:
            continue
        blocked.append(f"{path}  (uid={st.st_uid}, mode={oct(st.st_mode & 0o777)})")

    # /tmp/iceoryx2: the discovery directory must be enter-and-writable. If
    # it exists at all and we cannot use it, NodeBuilder fails the same way.
    iox_dir = Path("/tmp/iceoryx2")
    if iox_dir.is_dir() and not os.access(iox_dir, os.W_OK | os.X_OK):
        try:
            st = iox_dir.stat()
            blocked.append(f"{iox_dir}  (uid={st.st_uid}, mode={oct(st.st_mode & 0o777)})")
        except OSError:
            blocked.append(str(iox_dir))

    if not blocked:
        return

    sys.stderr.write(
        "\n"
        "ERROR: iceoryx2 shared-memory state is owned by another user.\n"
        "  raccoon's iox2 transport would now panic inside Rust with\n"
        "  NodeCreationFailure(InternalError) — an unrecoverable abort.\n"
        "\n"
        "Blocked paths:\n  " + "\n  ".join(blocked) + "\n"
        "\n"
        "Cleanup on the host (as root):\n"
        "  sudo systemctl stop stm32_data_reader.service 2>/dev/null || true\n"
        "  sudo rm -f /dev/shm/iox2_*\n"
        "  sudo rm -rf /tmp/iceoryx2\n"
        "  sudo systemctl start stm32_data_reader.service 2>/dev/null || true\n"
        "\n"
        "Root cause: some process previously ran iceoryx2 as root (a sudo\n"
        "invocation of code that imports raccoon, a root-mode deploy step,\n"
        "etc.). After cleanup, ensure iox2-using code only runs as the\n"
        "intended service user.\n"
    )
    sys.stderr.flush()
    raise SystemExit(2)


_check_iceoryx2_shm_access()

with contextlib.suppress(RuntimeError):
    get_transport()
atexit.register(shutdown_transport)


# UI library (replaces legacy RenderScreen)
from raccoon.ui import *
from raccoon.drive import Drive, AxisVelocityControlConfig, ChassisVelocityControlConfig
from raccoon.motion import UnifiedMotionPidConfig
from raccoon.kinematics_mecanum import MecanumKinematics
from raccoon.foundation import (
    Feedforward,
    FeedforwardController,
    MotorCalibration,
    PidConfig,
    PidController,
    PidGains,
)
from raccoon.motion import AxisConstraints
from raccoon.hal import IMU
from raccoon.kinematics_differential import DifferentialKinematics
from raccoon.kmeans import KMeans, KMeansResult
from raccoon.map import WorldMap as TableMap, MapSegment
from raccoon.localization import (
    Localization,
    LocalizationConfig,
    Observation,
    SurfaceKind,
    SurfaceMeasurement,
)

__all__ = [
    # Core hardware
    "Motor",
    "Servo",
    "ServoPreset",
    "ArmPreset",
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
    # Timing
    "StepTimingTracker",
    "get_transport",
    "shutdown_transport",
    # UI
    "UIStep",
    "UIScreen",
    # Drive & Kinematics
    "Drive",
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
    # Localization
    "Localization",
    "LocalizationConfig",
    "Observation",
    "SurfaceKind",
    "SurfaceMeasurement",
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
]
# Re-export everything from the bundled subpackages so callers can do
# ``from raccoon import drive_forward`` without juggling submodule paths.
# Splatting the upstream ``__all__`` lists into the literal would trip
# PLE0604 (``__all__`` must contain string literals at module scope), so
# extend the list explicitly.
__all__ += list(_robot_all)
__all__ += list(_step_all)
__all__ += list(_ui_all)

_PREVIOUS_SIGNAL_HANDLERS: dict[int, signal.Handlers] = {}


class _ShutdownHookState:
    """Tracks one-shot installation of atexit + signal handlers.

    Wrapping the toggle as a class attribute lets ``_install_shutdown_hooks``
    flip the flag without ``global``, so the function stays self-contained
    and the linter does not flag the assignment.
    """

    installed: bool = False


class _StartupState:
    """Tracks one-shot execution of logging init + startup banner."""

    done: bool = False


def _disable_all_motors() -> None:
    """Best-effort motor disarm during process exit.

    Logged failures matter: a robot whose Python-side disarm threw silently
    is a robot that may keep driving after the mission ends. The C++ HAL
    still has its own atexit-registered fallback, so this layer is allowed
    to fail loudly without the process refusing to exit.
    """
    disable = getattr(Motor, "disable_all", None)
    if disable is None:
        return

    try:
        info("Disabling all motors")
        disable()
    except Exception as e:
        # ``error()`` itself routes through spdlog; if logging is already
        # half-torn-down, fall back to stderr via os.write so the message
        # isn't lost. ``sys.stderr.write`` would also work but os.write
        # avoids re-entering Python's IO layer during interpreter teardown.
        try:
            error(f"Failed to disable all motors via Motor.disable_all: {e!r}")
        except Exception:
            import os

            os.write(2, f"raccoon: motor disarm failed: {e!r}\n".encode())


def _shutdown_logging() -> None:
    """Tear down spdlog before C++ static destruction begins.

    Failure here is non-fatal but worth surfacing — a stuck logger means
    later C++ destructors may attempt to log into a dead sink.
    """
    try:
        foundation.shutdown_logging()
    except Exception as e:
        import os

        os.write(2, f"raccoon: logging shutdown failed: {e!r}\n".encode())


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
    """Register atexit + SIGINT/SIGTERM hooks that disable all motors.

    Idempotent. Safe to call from non-main threads — ``signal.signal()`` is
    skipped with a debug log when it raises ``ValueError`` ("only from main
    thread"), so importing raccoon from a worker thread does not crash.
    """
    if _ShutdownHookState.installed:
        return

    _ShutdownHookState.installed = True

    atexit.register(_shutdown_logging)
    atexit.register(_disable_all_motors)
    debug("Registered atexit shutdown hook to disable all motors")

    for sig_name in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue

        try:
            previous = signal.getsignal(sig)
        except (OSError, ValueError) as e:
            # OSError: invalid signal on this platform; ValueError: not
            # called from main thread. Both mean we should skip this
            # signal but keep installing the rest.
            debug(f"Skipping shutdown handler for {sig_name}: {e!r}")
            continue

        _PREVIOUS_SIGNAL_HANDLERS[sig] = previous

        def handler(signum: int, frame: FrameType | None) -> None:
            _disable_all_motors()
            _forward_signal(signum, frame)

        try:
            signal.signal(sig, handler)
        except (OSError, ValueError) as e:
            # signal.signal() also enforces "main thread only" and may
            # reject the call on platforms where the signal exists but
            # cannot be re-handled. Log and skip — the C++ HAL still has
            # its own atexit-registered fallback.
            _PREVIOUS_SIGNAL_HANDLERS.pop(sig, None)
            debug(f"Skipping shutdown handler for {sig_name}: {e!r}")
            continue

        debug(f"Registered signal handler for {sig_name}")


def _log_startup_banner() -> None:
    """Emit the version banner via the logger, not stdout.

    Side note: this used to be a top-level ``print(...)``, which broke any
    tool that imported raccoon and parsed stdout (CLI JSON output, doctest
    runners, raccoon-cli subcommands). Routing it through the logger means
    consumers can silence it by raising the log level.
    """
    no_cal_label = " | --no-calibrate" if _is_no_calibrate() else ""
    no_chk_label = " | --no-checkpoints" if _is_no_checkpoints() else ""
    info(
        f"raccoon v{__version__} | raccoon-transport v{_raccoon_version}"
        f"{no_cal_label}{no_chk_label}"
    )


def _startup_init() -> None:
    """Initialize logging, print banner, and install shutdown hooks.

    Idempotent. Called by GenericRobot.__init__ so these side effects are
    deferred until a robot is actually instantiated, not on every import.
    """
    if _StartupState.done:
        return
    _StartupState.done = True
    initialize_logging()
    _log_startup_banner()
    _install_shutdown_hooks()
