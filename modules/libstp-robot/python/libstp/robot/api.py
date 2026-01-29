from abc import ABC, abstractmethod
from typing import Any, List, Optional, Protocol, TYPE_CHECKING, runtime_checkable
import asyncio

from libstp.class_name_logger import ClassNameLogger
from libstp.hal import AnalogSensor, DigitalSensor
from libstp.timing import Synchronizer
from libstp.foundation import initialize_timer

if TYPE_CHECKING:
    from libstp.drive import Drive
    from libstp.mission.api import MissionProtocol
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

class GenericRobot(ABC, ClassNameLogger):
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
    def missions(self) -> List["MissionProtocol"]:
        """List of missions to execute. Override to provide missions."""
        return []

    @property
    def setup_mission(self) -> Optional["MissionProtocol"]:
        """Optional mission to run before main missions."""
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
        # Initialize button from defs
        from libstp import button
        button.set_digital(self.defs.button)

        if not self.missions:
            self.warn("Robot does not have any missions attached")

        if self.setup_mission is not None:
            self.info("Setup mission found")

        if self.shutdown_mission is not None:
            self.info("Shutdown mission found")

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

    async def _run_missions(self) -> None:
        """Internal mission execution loop."""
        initialize_timer() # reset clock to 0
        if self.setup_mission is not None:
            self.info("Running setup mission")
            await self.setup_mission.run(self)

        initialize_timer() # reset clock to 0 before main missions
        self.synchronizer.start_recording()
        for mission in self.missions:
            self.info(f"Starting mission: {mission}")
            await mission.run(self)
            self.info(f"Finished mission: {mission}")

        initialize_timer() # reset clock to 0 before shutdown mission
        if self.shutdown_mission is not None:
            self.info("Running shutdown mission")
            await self.shutdown_mission.run(self)
