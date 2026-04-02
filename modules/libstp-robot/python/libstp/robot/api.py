from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Protocol, Type, TypeVar, TYPE_CHECKING, runtime_checkable
import asyncio

from libstp.class_name_logger import ClassNameLogger
from libstp.hal import AnalogSensor, DigitalSensor
from libstp.motion import UnifiedMotionPidConfig
from libstp.timing import Synchronizer
from libstp.foundation import initialize_timer
from .geometry import RobotGeometry

from .service import RobotService

_S = TypeVar("_S", bound=RobotService)

if TYPE_CHECKING:
    from libstp.drive import Drive
    from libstp.mission.api import MissionProtocol, SetupMission
    from libstp.odometry import Odometry


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

    analog_sensors: List[AnalogSensor]
    button: DigitalSensor
    wait_for_light_sensor: Optional[AnalogSensor]
    wait_for_light_mode: str  # "auto" (default) or "legacy"
    wait_for_light_drop_fraction: float  # sensitivity for auto mode (default 0.15)

class GenericRobot(ABC, RobotGeometry, ClassNameLogger):
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
    @abstractmethod
    def odometry(self) -> "Odometry":
        """Odometry system for position tracking."""
        ...

    @property
    @abstractmethod
    def shutdown_in(self) -> float:
        """Maximum runtime in seconds for main missions. MUST be specified."""
        ...

    @property
    def motion_pid_config(self) -> UnifiedMotionPidConfig:
        """Unified PID configuration for all motion primitives. Override to customize."""
        return UnifiedMotionPidConfig()

    @property
    def missions(self) -> List["MissionProtocol"]:
        """List of missions to execute. Override to provide missions."""
        return []

    @property
    def setup_mission(self) -> Optional["SetupMission"]:
        """Optional setup mission to run before main missions.

        Must be a ``SetupMission`` instance (not a plain ``Mission``).
        """
        return None

    @property
    def shutdown_mission(self) -> Optional["MissionProtocol"]:
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
        self._services: Dict[Type[RobotService], RobotService] = {}

        # Clear STM32 shutdown flag to enable motors and servos
        from libstp.hal import Motor
        Motor.enable_all()

        # Initialize button from defs
        from libstp import button
        button.set_digital(self.defs.button)

        # Auto-configure wheel speed desaturation from motion config
        if hasattr(self.kinematics, 'set_max_wheel_speed') and hasattr(self, 'motion_pid_config'):
            cfg = self.motion_pid_config
            max_linear = max(cfg.linear.max_velocity, cfg.lateral.max_velocity)
            max_wheel = max_linear / self.kinematics.get_wheel_radius()
            self.kinematics.set_max_wheel_speed(max_wheel)
            self.info(f"Kinematics desaturation: max_wheel_speed={max_wheel:.2f} rad/s")

        if not self.missions:
            self.warn("Robot does not have any missions attached")

        if self.setup_mission is not None:
            from libstp.mission.api import SetupMission
            if not isinstance(self.setup_mission, SetupMission):
                raise TypeError(
                    f"setup_mission must be a SetupMission instance, "
                    f"got {type(self.setup_mission).__name__}. "
                    f"Subclass SetupMission instead of Mission for setup missions."
                )
            self.info("Setup mission found")

        if self.shutdown_mission is not None:
            self.info("Shutdown mission found")

    def get_service(self, cls: Type[_S]) -> _S:
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
            from libstp.step import wait_for_button
            await wait_for_button().run_step(self)
        else:
            sensor = getattr(self.defs, "wait_for_light_sensor", None)
            if sensor is None:
                self.warn("No wait_for_light_sensor configured in defs! Falling back to wait_for_button.")
                from libstp.step import wait_for_button
                await wait_for_button().run_step(self)
            else:
                self.info("Waiting for light...")
                wfl_mode = getattr(self.defs, "wait_for_light_mode", "auto")
                if wfl_mode == "legacy":
                    from libstp.step import wait_for_light_legacy
                    await wait_for_light_legacy(sensor).run_step(self)
                else:
                    from libstp.step import wait_for_light
                    drop_fraction = getattr(self.defs, "wait_for_light_drop_fraction", 0.15)
                    await wait_for_light(sensor, drop_fraction=drop_fraction).run_step(self)

    async def _run_main_missions(self, missions: List["MissionProtocol"]) -> None:
        """Execute main missions sequentially."""
        for mission in missions:
            self.info(f"Starting mission: {mission}")
            await mission.run(self)
            self.info(f"Finished mission: {mission}")
            self._warn_leaked_background_tasks(mission)

    def _warn_leaked_background_tasks(self, mission: "MissionProtocol") -> None:
        """Warn if background tasks from a mission are still running."""
        bg_mgr = getattr(self, "_background_manager", None)
        if bg_mgr is not None and bg_mgr.active_count > 0:
            self.warning(
                f"{bg_mgr.active_count} background task(s) still running "
                f"after mission {mission} — consider using wait_for_background()"
            )

    async def _drain_background_tasks(self) -> None:
        """Cancel all remaining background tasks and await their cleanup."""
        bg_mgr = getattr(self, "_background_manager", None)
        if bg_mgr is None or bg_mgr.active_count == 0:
            return
        self.info(
            f"Cancelling {bg_mgr.active_count} remaining background task(s) "
            f"before shutdown"
        )
        await bg_mgr.cancel_all()

    async def _run_missions(self) -> None:
        """Internal mission execution loop."""
        missions = list(self.missions)
        setup_mission = self.setup_mission
        shutdown_mission = self.shutdown_mission

        self._preload_missions(missions, setup_mission, shutdown_mission)

        initialize_timer() # reset clock to 0
        if setup_mission is not None:
            self.info("Running setup mission")
            await setup_mission.run(self)

        # Pre-start gate (wait for light or button)
        if setup_mission is not None and setup_mission._custom_pre_start_gate:
            self.info("Using setup mission's custom pre-start gate")
            await setup_mission.pre_start_gate(self)
        else:
            await self._pre_start_gate()

        # Main missions with shutdown timer
        initialize_timer() # reset clock to 0 before main missions
        self.synchronizer.start_recording()

        try:
            await asyncio.wait_for(
                self._run_main_missions(missions),
                timeout=self.shutdown_in,
            )
        except asyncio.TimeoutError:
            self.error(f"Shutdown timer expired after {self.shutdown_in}s! Forcing shutdown.")

        # Cancel orphaned background tasks before shutdown mission
        await self._drain_background_tasks()

        # Always run shutdown mission
        initialize_timer() # reset clock to 0 before shutdown mission
        if shutdown_mission is not None:
            self.info("Running shutdown mission")
            await shutdown_mission.run(self)

        self.info("Robot execution complete. Exiting.")

    def _preload_missions(
        self,
        missions: List["MissionProtocol"],
        setup_mission: Optional["MissionProtocol"],
        shutdown_mission: Optional["MissionProtocol"],
    ) -> None:
        """Build mission sequences early to validate step construction."""
        preload_targets: List[tuple[str, "MissionProtocol"]] = []
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
            sequence_builder()
