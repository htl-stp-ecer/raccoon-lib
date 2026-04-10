from abc import abstractmethod
from typing import List, Optional, runtime_checkable, Protocol, TYPE_CHECKING

from raccoon.class_name_logger import ClassNameLogger
from raccoon.foundation import initialize_timer

if TYPE_CHECKING:
    from raccoon.step.base import Step


@runtime_checkable
class MissionProtocol(Protocol):
    """Protocol for mission objects that can be run on a robot."""

    async def run(self, robot: "GenericRobot") -> None:
        """Execute the mission on the given robot."""
        ...


class Mission(ClassNameLogger, MissionProtocol):
    """Base mission that delegates execution to a root step sequence.

    Subclasses may set ``time_budget`` (seconds) as a class attribute to
    arm a per-mission deadline. If the mission exceeds its budget the
    robot's ``WatchdogManager`` fires, cancels the main-mission task, and
    routes through the normal shutdown path — the shutdown mission still
    runs. Subsequent missions do not run after a budget expiry.
    """

    time_budget: Optional[float] = None

    def __str__(self):
        return self.__class__.__name__

    def __repr__(self):
        return self.__class__.__name__

    async def run(self, robot):
        """Build the mission sequence and execute it on the provided robot."""
        self.info(f"Starting mission: {self.__class__.__name__}")
        await self.sequence().run_step(robot)
        self.info(f"Completed mission: {self.__class__.__name__}")

    @abstractmethod
    def sequence(self) -> "Step":
        """Return the root step tree for this mission."""
        raise NotImplementedError("Method sequence() not implemented")


class SetupMission(Mission):
    """Base class for setup missions with a customizable pre-start gate.

    Subclass this instead of ``Mission`` for setup missions. Override
    ``pre_start_gate()`` to customize what happens between the setup
    sequence and the main missions (e.g. skip waiting for light/button).

    Example::

        class MySetup(SetupMission):
            def sequence(self) -> Step:
                return seq(calibrate_deadzone())

            async def pre_start_gate(self, robot) -> None:
                # Skip the wait-for-light — just wait for button
                from raccoon.step import wait_for_button
                await wait_for_button().run_step(robot)

        class QuickSetup(SetupMission):
            def sequence(self) -> Step:
                return seq(calibrate_deadzone())

            async def pre_start_gate(self, robot) -> None:
                pass  # Skip all waiting — start immediately
    """

    _custom_pre_start_gate: bool = False

    async def pre_start_gate(self, robot) -> None:
        """Gate executed after the setup sequence and before main missions.

        The default delegates to the robot's built-in wait-for-light /
        wait-for-button logic. Override this to customize or skip the
        gate entirely.

        To skip all waiting::

            async def pre_start_gate(self, robot) -> None:
                pass  # start immediately
        """

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if "pre_start_gate" in cls.__dict__:
            cls._custom_pre_start_gate = True