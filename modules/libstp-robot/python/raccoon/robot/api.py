from __future__ import annotations

import asyncio
import contextlib
from abc import ABC, abstractmethod
from typing import (
    TYPE_CHECKING,
    Protocol,
    TypeVar,
    runtime_checkable,
)

from raccoon.class_name_logger import ClassNameLogger
from raccoon.foundation import initialize_timer
from raccoon.hal import AnalogSensor, DigitalSensor
from raccoon.motion import UnifiedMotionPidConfig
from raccoon.timing import Synchronizer

from .geometry import RobotGeometry
from .service import RobotService

_S = TypeVar("_S", bound=RobotService)

# Sentinel distinct from None so the localization property can tell apart
# "never touched" (auto-wire on access) from "explicitly set to None"
# (treated as a wiring error so silent-None can't sneak past callers).
_AUTO_WIRE_LOCALIZATION = object()

# === TESTING OVERRIDE ===================================================
# Hard kill-switch for test runs. When True, localization runs in pure
# dead-reckoning mode:
#   * the C++ ``Localization`` particle filter is NEVER constructed, so its
#     background predict thread never spawns and no particle-filter math runs;
#   * the continuous line-sensor → filter fusion loop never starts, so there
#     is no fusion thread and no ``observe()``/particle weighting.
# Steps that read ``robot.localization`` still get a live pose straight from
# odometry (``get_pose()`` forwards odometry; ``observe()`` is a no-op).
# Flip back to False to restore real Monte-Carlo localization.
_DISABLE_LOCALIZATION_FOR_TESTING = True


class _DeadReckoningLocalization:
    """Testing stub used when ``_DISABLE_LOCALIZATION_FOR_TESTING`` is set.

    Presents the two methods every consumer uses (``get_pose`` / ``observe``)
    without a particle filter or any background thread: ``get_pose`` forwards
    the live odometry pose and ``observe`` discards the observation. Odometry
    is read lazily so construction never triggers hardware access.
    """

    def __init__(self, robot: "GenericRobot") -> None:
        self._robot = robot

    def get_pose(self):
        return self._robot.odometry.get_pose()

    def observe(self, *args, **kwargs) -> None:
        return None


class LocalizationNotWiredError(RuntimeError):
    """Raised when ``robot.localization`` cannot be auto-wired.

    The underlying cause (ImportError, odometry build failure, Localization
    constructor exception, ...) is chained via ``__cause__``. Catch this in
    robot subclasses that legitimately have no localization (mock/headless
    setups) and override the property.
    """


def _format_odometry_source(source) -> str:
    """Render a pybind ``OdometrySource`` enum or fallback value for logs."""
    name = getattr(source, "name", None)
    if isinstance(name, str) and name:
        return name
    text = str(source)
    if "." in text:
        return text.rsplit(".", 1)[-1]
    return text


if TYPE_CHECKING:
    from raccoon.drive import Drive
    from raccoon.hal import IOdometry as Odometry
    from raccoon.localization import Localization
    from raccoon.map import WorldMap as TableMap
    from raccoon.mission.api import MissionProtocol, SetupMission
    from raccoon.robot.localization_fusion import ContinuousLocalizationFusion


@runtime_checkable
class RobotDefinitionsProtocol(Protocol):
    """
    Protocol defining the structure of robot hardware definitions.

    Implementations should define motor and servo attributes as class variables.
    Example:
        class MyDefs:
            left_motor = Motor(port=0, ...)
            right_motor = Motor(port=1, ...)
            arm_servo = Servo(port=0, ...)

    Drive motors are accessed via robot.drive.get_motors() instead of defs.
    """

    analog_sensors: list[AnalogSensor]
    button: DigitalSensor
    wait_for_light_sensor: AnalogSensor | None
    wait_for_light_mode: str  # "auto" (default) or "legacy"
    wait_for_light_drop_fraction: float  # sensitivity for auto mode (default 0.15)


class GenericRobot(ABC, RobotGeometry, ClassNameLogger):
    """
    Abstract base class for all robots.

    Subclasses must implement:
        - defs: Hardware definitions (motors, servos, etc.)
        - drive: Drive system for chassis control

    The ``odometry`` property is concrete and lazily built from the platform
    factory; subclasses may override it but don't have to.

    Optional attributes:
        - missions: List of missions to execute
        - setup_mission: Mission to run before main missions
        - shutdown_mission: Mission to run after all missions complete
    """

    @property
    @abstractmethod
    def defs(self) -> RobotDefinitionsProtocol:
        """Hardware definitions (motors, servos, sensors)."""
        ...

    @property
    @abstractmethod
    def drive(self) -> "Drive":
        """Drive system for chassis velocity control."""
        ...

    @property
    def odometry(self) -> "Odometry":
        """Odometry system for position tracking.

        Lazily constructed on first access via the platform factory
        ``Platform.create_odometry(kinematics)``, which returns the
        platform-canonical ``IOdometry`` implementation. Subclasses may
        override this property (e.g. for tests) but no longer have to.
        """
        if not hasattr(self, "_odometry"):
            from raccoon.hal import platform as _platform

            self._odometry = _platform.Platform.create_odometry(self.kinematics)
        return self._odometry

    @property
    @abstractmethod
    def shutdown_in(self) -> float:
        """Maximum runtime in seconds for main missions. MUST be specified."""
        ...

    @property
    def table_map(self) -> "TableMap" | None:
        """Table map with field line/wall geometry. ``None`` if not configured."""
        return None

    @property
    def localization(self) -> "Localization":
        """World-pose localization service. Always wired, never ``None``.

        Lazily constructed on first access by wrapping ``self.odometry`` (and
        ``self.table_map`` if available). The instance is cached in
        ``self._localization``; a subclass that already populated that field
        wins, so manual wiring still works.

        Raises:
            LocalizationNotWiredError: When auto-wiring cannot proceed —
                the ``raccoon.localization`` extension is missing, the
                odometry property failed to build, or the ``Localization``
                constructor itself threw. The original exception is chained
                via ``__cause__``.

        To disable in a subclass (e.g. headless tests), override this
        property to raise. Setting ``self._localization = None`` explicitly
        is treated as a wiring error so silent-None can't sneak past
        callers.
        """
        cached = getattr(self, "_localization", _AUTO_WIRE_LOCALIZATION)
        if cached is _AUTO_WIRE_LOCALIZATION:
            self._localization = self._build_localization()
            return self._localization
        if cached is None:
            msg = (
                "robot._localization is explicitly None. localization is a "
                "hard requirement — to disable, override the localization "
                "property instead of zeroing the cache field."
            )
            raise LocalizationNotWiredError(msg)
        return cached

    def _build_localization(self) -> "Localization":
        """Build the platform-default Localization wrapping ``self.odometry``."""
        # TESTING: skip the real particle filter entirely — no C++ predict
        # thread, no particle-filter computation. See the module-level note.
        if _DISABLE_LOCALIZATION_FOR_TESTING:
            self.info(
                "Localization DISABLED for testing — dead-reckoning only (no particle filter, no thread)"
            )
            return _DeadReckoningLocalization(self)

        try:
            from raccoon.localization import Localization, LocalizationConfig
        except ImportError as exc:
            msg = (
                "raccoon.localization is not importable — the particle-filter "
                "C++ extension is missing from this build. Rebuild the wheel "
                "(deploy.sh / build.sh) or override the localization property."
            )
            raise LocalizationNotWiredError(msg) from exc

        try:
            odom = self.odometry
        except Exception as exc:
            msg = (
                "Cannot auto-wire robot.localization: robot.odometry raised "
                "during construction. Fix the odometry path or override the "
                "localization property to supply a service manually."
            )
            raise LocalizationNotWiredError(msg) from exc
        if odom is None:
            msg = (
                "Cannot auto-wire robot.localization: robot.odometry returned "
                "None. Localization needs a live IOdometry source."
            )
            raise LocalizationNotWiredError(msg)

        try:
            table_map = self.table_map
        except Exception as exc:
            msg = (
                "Cannot auto-wire robot.localization: robot.table_map raised. "
                "Surface-measurement likelihoods need a valid map (or None)."
            )
            raise LocalizationNotWiredError(msg) from exc

        try:
            cfg = LocalizationConfig()
            loc = Localization(odom, cfg, table_map=table_map)
        except Exception as exc:
            msg = (
                f"Localization() constructor failed: {exc!r}. Check the "
                f"odometry instance type and the table_map (if set)."
            )
            raise LocalizationNotWiredError(msg) from exc

        # Best-effort wiring of the post-run recorder. The env-var policy
        # lives in C++ (raccoon.localization._auto_enable_recording); we
        # just hand it a robot/sensor snapshot for the JSON header. Any
        # failure here is a debug-feature loss, not a runtime fault.
        try:
            self._wire_localization_recording(loc, cfg)
        except Exception as exc:  # pragma: no cover - defensive
            import logging

            logging.getLogger(__name__).warning("Failed to wire localization recorder: %r", exc)
        return loc

    def _wire_localization_recording(self, loc: "Localization", cfg) -> None:
        """Auto-enable the post-run recorder when env vars are set.

        Builds a small JSON snapshot of robot dimensions + sensor offsets so
        the Web-IDE replay panel can render the chassis without needing a
        live robot definition. Failures are non-fatal — recording is a debug
        feature, never required for robot operation.
        """
        import json

        try:
            from raccoon.localization import _auto_enable_recording
        except ImportError:
            return  # older C++ binding without the helper

        width_cm = float(getattr(self, "width_cm", 0.0) or 0.0)
        length_cm = float(getattr(self, "length_cm", 0.0) or 0.0)
        sensor_entries: list[dict] = []
        try:
            all_sensors = self.all_sensors() if callable(getattr(self, "all_sensors", None)) else {}
        except Exception:
            all_sensors = {}
        for sensor, pos in all_sensors.items():
            name = getattr(sensor, "name", None) or type(sensor).__name__
            sensor_entries.append(
                {
                    "name": str(name),
                    "kind": "sensor",
                    "forward_cm": float(getattr(pos, "forward_cm", 0.0)),
                    "strafe_cm": float(getattr(pos, "strafe_cm", 0.0)),
                }
            )
        robot_json = json.dumps({"width_cm": width_cm, "length_cm": length_cm})
        sensors_json = json.dumps(sensor_entries)
        tick_hz = 1000.0 / max(int(getattr(cfg, "tick_period_ms", 10) or 10), 1)
        particle_count_hint = int(getattr(cfg, "particle_count", 128) or 128)
        _auto_enable_recording(
            loc,
            robot_json=robot_json,
            sensors_json=sensors_json,
            table_map_json="null",
            particle_count_hint=particle_count_hint,
            tick_hz=tick_hz,
        )

    @property
    def motion_pid_config(self) -> UnifiedMotionPidConfig:
        """Unified PID configuration for all motion primitives. Override to customize."""
        return UnifiedMotionPidConfig()

    @property
    def missions(self) -> list["MissionProtocol"]:
        """List of missions to execute. Override to provide missions."""
        return []

    @property
    def setup_mission(self) -> "SetupMission" | None:
        """Optional setup mission to run before main missions.

        Must be a ``SetupMission`` instance (not a plain ``Mission``).
        """
        return None

    @property
    def shutdown_mission(self) -> "MissionProtocol" | None:
        """Optional mission to run after all missions complete."""
        return None

    @property
    def synchronizer(self) -> Synchronizer:
        """Synchronizer for coordinating async tasks."""
        if not hasattr(self, "_synchronizer"):
            self._synchronizer = Synchronizer()
        return self._synchronizer

    def __init__(self) -> None:
        """Initialize the robot and log configuration status."""
        import raccoon as _raccoon_pkg

        _raccoon_pkg._startup_init()

        self._services: dict[type[RobotService], RobotService] = {}

        # Clear STM32 shutdown flag to enable motors and servos
        from raccoon.hal import Motor

        Motor.enable_all()

        # Initialize button from defs
        from raccoon import button

        button.set_digital(self.defs.button)

        # Auto-configure wheel speed desaturation from motion config
        if hasattr(self.kinematics, "set_max_wheel_speed") and hasattr(self, "motion_pid_config"):
            cfg = self.motion_pid_config
            max_linear = max(cfg.linear.max_velocity, cfg.lateral.max_velocity)
            max_wheel = max_linear / self.kinematics.get_wheel_radius()
            self.kinematics.set_max_wheel_speed(max_wheel)
            self.info(f"Kinematics desaturation: max_wheel_speed={max_wheel:.2f} rad/s")

        # Re-apply the MCU chassis velocity-command gains calibrated by the
        # autotune velocity phase (Phase 6). They are persisted under
        # robot.drive.kinematics.velocity_command_gain and folded into the STM32
        # forward kinematics, so they must be set BEFORE odometry is first
        # created (which pushes the kinematics config to the coprocessor).
        if hasattr(self.kinematics, "set_velocity_command_gains"):
            try:
                from raccoon.project_yaml import find_project_root, read_project_value

                root = find_project_root()
                gains = (
                    read_project_value(
                        root, ["robot", "drive", "kinematics", "velocity_command_gain"], None
                    )
                    if root is not None
                    else None
                )
                if isinstance(gains, dict):
                    gx = float(gains.get("vx", 1.0))
                    gy = float(gains.get("vy", 1.0))
                    gw = float(gains.get("wz", 1.0))
                    self.kinematics.set_velocity_command_gains(gx, gy, gw)
                    self.info(
                        f"Kinematics velocity-command gains: vx={gx:.4f} vy={gy:.4f} wz={gw:.4f}"
                    )
            except Exception as exc:
                self.warn(f"Could not apply velocity_command_gain from config: {exc}")

        if not self.missions:
            self.warn("Robot does not have any missions attached")

        try:
            source = self.odometry.get_active_source()
            self.info(f"Odometry source at startup: {_format_odometry_source(source)}")
        except Exception as exc:
            self.warn(f"Could not determine startup odometry source: {exc}")

        if self.setup_mission is not None:
            from raccoon.mission.api import SetupMission

            if not isinstance(self.setup_mission, SetupMission):
                msg = (
                    f"setup_mission must be a SetupMission instance, "
                    f"got {type(self.setup_mission).__name__}. "
                    f"Subclass SetupMission instead of Mission for setup missions."
                )
                raise TypeError(msg)
            self.info("Setup mission found")

        if self.shutdown_mission is not None:
            self.info("Shutdown mission found")

    def get_service(self, cls: type[_S]) -> _S:
        """Get or create a cached service instance.

        Services are lazily instantiated on first access and reused for
        subsequent calls with the same class.
        """
        service = self._services.get(cls)
        if service is None:
            service = cls(self)
            self._services[cls] = service
        return service  # type: ignore[return-value]

    def start(self) -> None:
        """
        Start executing the robot's missions.

        Runs setup_mission (if present), then all missions in order,
        then shutdown_mission (if present).

        Note: This method blocks until all missions complete.
        For non-blocking execution, use start_async() instead.
        """
        self.info("Starting robot")
        asyncio.run(self._run_missions())

    async def start_async(self) -> None:
        """
        Async version of start() for use in existing event loops.

        Useful for testing or integration with other async code.
        """
        self.info("Starting robot (async)")
        await self._run_missions()

    async def _pre_start_gate(self) -> None:
        """Wait for light or button press before starting main missions.

        Override this method in a subclass to customize pre-start behavior.
        """
        import os

        dev_mode = os.environ.get("LIBSTP_DEV_MODE") == "1"

        if dev_mode:
            self.info("Dev mode: waiting for button press")
            from raccoon.step import wait_for_button

            await wait_for_button("Start by clicking the button").run_step(self)
        else:
            sensor = getattr(self.defs, "wait_for_light_sensor", None)
            if sensor is None:
                self.warn(
                    "No wait_for_light_sensor configured in defs! Falling back to wait_for_button."
                )
                from raccoon.step import wait_for_button

                await wait_for_button().run_step(self)
            else:
                self.info("Waiting for light...")
                wfl_mode = getattr(self.defs, "wait_for_light_mode", "auto")
                if wfl_mode == "legacy":
                    from raccoon.step import wait_for_light_legacy

                    await wait_for_light_legacy(sensor).run_step(self)
                else:
                    from raccoon.step import wait_for_light

                    drop_fraction = getattr(self.defs, "wait_for_light_drop_fraction", 0.15)
                    await wait_for_light(sensor, drop_fraction=drop_fraction).run_step(self)

    async def _run_main_missions(self, missions: list["MissionProtocol"]) -> None:
        """Execute main missions sequentially with per-mission watchdogs."""
        from raccoon.step.watchdog_manager import get_watchdog_manager

        wdt = get_watchdog_manager(self)
        for mission in missions:
            self.info(f"Starting mission: {mission}")

            budget = getattr(mission, "time_budget", None)
            wd_name: str | None = None
            if budget is not None and budget > 0:
                wd_name = f"mission:{mission}"
                wdt.arm(wd_name, float(budget), source="mission")

            try:
                await mission.run(self)
            finally:
                if wd_name is not None:
                    wdt.disarm(wd_name, missing_ok=True)

            self.info(f"Finished mission: {mission}")
            self._warn_leaked_background_tasks(mission)
            self._warn_leaked_watchdogs(mission)

    def _warn_leaked_watchdogs(self, mission: "MissionProtocol") -> None:
        """Warn if user watchdogs from a mission are still armed at its end."""
        from raccoon.step.watchdog_manager import get_watchdog_manager

        wdt = get_watchdog_manager(self)
        leaked = wdt.active_names(source="user")
        if leaked:
            self.warn(
                f"{len(leaked)} watchdog(s) still armed after mission {mission}: "
                f"{', '.join(leaked)} — call stop_watchdog() before the mission ends"
            )
            for name in leaked:
                wdt.disarm(name, missing_ok=True)

    def _warn_leaked_background_tasks(self, mission: "MissionProtocol") -> None:
        """Warn if background tasks from a mission are still running."""
        bg_mgr = getattr(self, "_background_manager", None)
        if bg_mgr is not None and bg_mgr.active_count > 0:
            self.warn(
                f"{bg_mgr.active_count} background task(s) still running "
                f"after mission {mission} — consider using wait_for_background()"
            )

    #: Run the continuous line-sensor → localization fusion loop during the main
    #: missions. Always-on by default (real Monte-Carlo localization); set to
    #: ``False`` on a robot subclass to fall back to dead-reckoning + resync-only.
    enable_localization_fusion: bool = True

    def _start_localization_fusion(self) -> "ContinuousLocalizationFusion | None":
        """Launch the always-on line-sensor → localization fuser, or ``None``.

        The fuser runs on its OWN dedicated daemon thread — all localization
        work (sensor reads, building observations, ``observe()``) is off the
        asyncio motion loop and can never block the hot path. Returns a stop-
        handle (call ``.stop()``) or ``None`` when disabled, when localization
        has no table map (nothing to correct against), or when the robot has no
        line sensors. Never raises — a fusion-wiring problem must not abort the
        run."""
        # TESTING: never spawn the fusion thread / run particle weighting.
        if _DISABLE_LOCALIZATION_FOR_TESTING:
            return None
        if not getattr(self, "enable_localization_fusion", True):
            return None
        try:
            from raccoon.robot.localization_fusion import start_localization_fusion

            # start_localization_fusion is a no-op (returns None) without line
            # sensors; without a table map the C++ weighting is itself a cheap
            # no-op, so no extra guard is needed here.
            fusion = start_localization_fusion(self)
            if fusion is not None:
                self.info("Continuous localization fusion: ON (dedicated thread → particle filter)")
            return fusion
        except Exception as exc:  # pragma: no cover - defensive
            self.warn(f"Could not start localization fusion: {exc!r}")
            return None

    async def _drain_background_tasks(self) -> None:
        """Cancel all remaining background tasks and await their cleanup."""
        bg_mgr = getattr(self, "_background_manager", None)
        if bg_mgr is None or bg_mgr.active_count == 0:
            return
        self.info(
            f"Cancelling {bg_mgr.active_count} remaining background task(s) " f"before shutdown"
        )
        await bg_mgr.cancel_all()

    def _force_quit(self, reason: str) -> None:
        """Last-resort shutdown when asyncio cancellation is ignored.

        Sets the STM32 shutdown flag and fully disables servos via the HAL
        (firmware-side stop is instant and survives process death), then
        terminates the process with ``os._exit`` to bypass any Python-level
        code still swallowing cancels. A brief sleep gives the LCM publisher
        a chance to flush the shutdown command before the process dies.
        """
        self.error(f"{reason} — forcing process exit")
        try:
            from raccoon.hal import Motor, Servo

            Motor.disable_all()
            Servo.fully_disable_all()
        except Exception as e:
            self.error(f"Failed to set STM32 shutdown flag: {e}")
        import time as _time

        _time.sleep(0.05)
        import os

        os._exit(2)

    @staticmethod
    def _get_skip_mission_indices() -> set:
        """Return mission order indices to skip, from LIBSTP_SKIP_MISSIONS env var."""
        import os

        val = os.environ.get("LIBSTP_SKIP_MISSIONS", "")
        if not val:
            return set()
        try:
            return {int(x) for x in val.split(",") if x.strip()}
        except ValueError:
            return set()

    def _run_platform_probe(self) -> None:
        """Run the hardware health-check before any mission code executes.

        Refuses to start when the STM32 bridge or IMU is unreachable. Set
        LIBSTP_SKIP_PROBE=1 to bypass (useful for offline development and
        headless tests where the LCM bridge is intentionally absent).
        """
        import os

        if os.environ.get("LIBSTP_SKIP_PROBE") == "1":
            self.info("LIBSTP_SKIP_PROBE=1 — skipping platform probe")
            return

        from raccoon.hal import platform as _platform

        try:
            result = _platform.Platform.probe()
        except Exception as e:
            self.error(f"Platform probe raised an unexpected error: {e!r}")
            raise

        for component in result.components:
            line = f"  - {component.component.name.lower()}: {'ok' if component.ok else 'FAIL'}"
            if component.detail:
                line += f" ({component.detail})"
            (self.info if component.ok else self.error)(line)

        if not result.ok:
            failed = ", ".join(c.component.name.lower() for c in result.failed_components())
            msg = f"Platform probe failed: {failed}\n{result.summary()}"
            raise _platform.ProbeFailedError(msg)

    async def _run_missions(self) -> None:
        """Internal mission execution loop."""
        all_missions = list(self.missions)
        skip_indices = self._get_skip_mission_indices()
        if skip_indices:
            skipped = [str(all_missions[i]) for i in sorted(skip_indices) if i < len(all_missions)]
            self.info(f"--no-m: skipping mission(s) at order {sorted(skip_indices)}: {skipped}")
        missions = [m for i, m in enumerate(all_missions) if i not in skip_indices]
        setup_mission = self.setup_mission
        shutdown_mission = self.shutdown_mission

        self._preload_missions(missions, setup_mission, shutdown_mission)

        # Hardware health-check before any code-driven motion. Confirms the
        # STM32 bridge, IMU and motor telemetry are reachable so missions don't
        # silently run on a half-attached robot. Skipped when LIBSTP_SKIP_PROBE=1
        # so headless test environments can opt out.
        self._run_platform_probe()

        initialize_timer()  # reset clock to 0
        if setup_mission is not None:
            from contextlib import nullcontext

            from raccoon.mission.api import SetupMission

            timer_ctx = (
                setup_mission.setup_timer_context()
                if isinstance(setup_mission, SetupMission)
                else nullcontext()
            )
            self.info("Running setup mission")
            async with timer_ctx:
                await setup_mission.run(self)
                # Pre-start gate runs inside the timer context so the WFL
                # screen (and any custom gate UI) still shows the countdown.
                if setup_mission._custom_pre_start_gate:
                    self.info("Using setup mission's custom pre-start gate")
                    await setup_mission.pre_start_gate(self)
                else:
                    await self._pre_start_gate()
        else:
            await self._pre_start_gate()

        # Main missions with shutdown timer and watchdog manager
        initialize_timer()  # reset clock to 0 before main missions
        self.synchronizer.start_recording()

        from raccoon.step.watchdog_manager import get_watchdog_manager

        wdt = get_watchdog_manager(self)

        # Continuous, automatic line-sensor → localization fusion: an always-on
        # loop that corrects the particle filter against the table map every tick
        # during the run, so localization is real Monte-Carlo localization rather
        # than dead-reckoning between rare resync steps. It runs on its OWN
        # daemon thread — never on this event loop — so it can never block the
        # motion hot path.
        fusion = self._start_localization_fusion()

        # Heading-drift watchdog: warns when the chassis rotates while no
        # motion step holds the drive (external contact during servo phases).
        # Own daemon thread, diagnostic only — see heading_drift_watchdog.py.
        drift_wd = None
        try:
            from raccoon.robot.heading_drift_watchdog import start_heading_drift_watchdog

            drift_wd = start_heading_drift_watchdog(self)
            if drift_wd is not None:
                self.info("Heading-drift watchdog: ON (dedicated thread)")
        except Exception as exc:  # pragma: no cover - defensive
            self.warn(f"Could not start heading-drift watchdog: {exc!r}")

        main_task = asyncio.create_task(self._run_main_missions(missions))
        wdt.attach_main_task(main_task)

        try:
            done, pending = await asyncio.wait([main_task], timeout=self.shutdown_in)
            if main_task in pending:
                self.error(f"Shutdown timer expired after {self.shutdown_in}s! Forcing shutdown.")
                main_task.cancel()
                with contextlib.suppress(TimeoutError, asyncio.CancelledError, Exception):
                    await asyncio.wait_for(asyncio.shield(main_task), timeout=2.0)

                if not main_task.done():
                    self._force_quit("Main task ignored cancellation after 2s grace period")
            elif main_task.cancelled():
                expired = wdt.expired_name
                if expired is not None:
                    self.error(
                        f"Watchdog '{expired}' expired — main missions cancelled, "
                        f"running shutdown mission"
                    )
                else:
                    self.error("Main missions cancelled externally")
            else:
                exc = main_task.exception()
                if exc is not None:
                    self.error(f"Main missions raised: {exc}")
        finally:
            wdt.detach_main_task()
            await wdt.cancel_all()
            if fusion is not None:
                # Join the fusion thread off the event loop so the (blocking)
                # join never stalls the loop; the thread is a daemon anyway.
                with contextlib.suppress(Exception):
                    await asyncio.to_thread(fusion.stop)
            if drift_wd is not None:
                with contextlib.suppress(Exception):
                    await asyncio.to_thread(drift_wd.stop)

        # Cancel orphaned background tasks before shutdown mission
        await self._drain_background_tasks()

        # Always run shutdown mission
        initialize_timer()  # reset clock to 0 before shutdown mission
        if shutdown_mission is not None:
            self.info("Running shutdown mission")
            await shutdown_mission.run(self)

        # Persist deferred diagnostics now that the robot is idle: the step-
        # timing DB and the per-mission profiler traces both buffer in RAM
        # during the run (so no fsync/json.dump lands between steps or missions)
        # and write once here.
        await self._flush_deferred_diagnostics()

        self.info("Robot execution complete. Exiting.")

    async def _flush_deferred_diagnostics(self) -> None:
        """Write the run's RAM-buffered timing + profiler traces after the run."""
        try:
            from raccoon.timing.tracker import StepTimingTracker

            if StepTimingTracker._instance is not None:
                await StepTimingTracker._instance.database.flush()
        except Exception as exc:  # never let diagnostics break shutdown
            self.debug(f"Step-timing flush skipped: {exc}")

        try:
            from raccoon.profiling import StepProfiler

            # json.dump is blocking; run it off the loop so the shutdown path
            # (and any UI ticking) stays responsive while traces land on disk.
            paths = await asyncio.to_thread(StepProfiler.flush_traces)
            if paths:
                self.info(f"Profiler: wrote {len(paths)} buffered trace(s) post-run")
        except ImportError:
            pass  # profiling add-on not installed
        except Exception as exc:
            self.debug(f"Profiler trace flush skipped: {exc}")

    def _preload_missions(
        self,
        missions: list["MissionProtocol"],
        setup_mission: "MissionProtocol" | None,
        shutdown_mission: "MissionProtocol" | None,
    ) -> None:
        """Build mission sequences early to validate step construction."""
        preload_targets: list[tuple[str, "MissionProtocol"]] = []
        if setup_mission is not None:
            preload_targets.append(("setup", setup_mission))
        preload_targets.extend(("main", mission) for mission in missions)
        if shutdown_mission is not None:
            preload_targets.append(("shutdown", shutdown_mission))

        if not preload_targets:
            return

        self.info("Preloading mission sequences")
        for label, mission in preload_targets:
            sequence_builder = getattr(mission, "sequence", None)
            if not callable(sequence_builder):
                self.warn(f"Mission does not expose sequence() for preload: {mission}")
                continue
            self.debug(f"Preloading {label} mission: {mission}")
            # Resolve the root so a bare StepBuilder at the top of sequence()
            # still triggers construction-time validation (nested builders are
            # already resolved via composite constructors).
            sequence_builder().resolve()
