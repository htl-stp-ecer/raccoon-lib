"""Watchdog DSL steps — arm, feed, and disarm named keepalive watchdogs.

Watchdogs are registered with the robot's ``WatchdogManager`` and, on
expiry, cancel the main-mission task. This triggers the normal shutdown
path: background tasks drain, the shutdown mission runs, then the process
exits.

Feeding is between-step only. Long-running steps should not attempt to
feed from inside their own update loop — use ``timeout(...)`` for
per-step stall detection instead.

Example::

    from raccoon.step import seq, start_watchdog, feed_watchdog, stop_watchdog
    from raccoon.step.motion import drive_forward

    seq(
        start_watchdog("scoring", timeout=5.0),
        drive_forward(30),
        feed_watchdog("scoring"),
        drive_forward(30),
        stop_watchdog("scoring"),
    )
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Union

from . import Step
from .annotation import dsl
from .watchdog_manager import get_watchdog_manager

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


DEFAULT_WATCHDOG_NAME = "default"


@dsl(hidden=True)
class StartWatchdog(Step):
    """Arm a named watchdog with a timeout."""

    def __init__(self, name: str, timeout: Union[float, int]) -> None:
        super().__init__()
        if not isinstance(name, str) or not name:
            raise ValueError(f"Watchdog name must be a non-empty string: {name!r}")
        if timeout <= 0:
            raise ValueError(f"Watchdog timeout must be positive: {timeout}")
        self._name = name
        self._timeout = float(timeout)

    def _generate_signature(self) -> str:
        return f"StartWatchdog(name={self._name!r}, timeout={self._timeout:.2f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        mgr = get_watchdog_manager(robot)
        mgr.arm(self._name, self._timeout, source="user")


@dsl(hidden=True)
class FeedWatchdog(Step):
    """Reset a named watchdog's deadline."""

    def __init__(self, name: str) -> None:
        super().__init__()
        if not isinstance(name, str) or not name:
            raise ValueError(f"Watchdog name must be a non-empty string: {name!r}")
        self._name = name

    def _generate_signature(self) -> str:
        return f"FeedWatchdog(name={self._name!r})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        mgr = get_watchdog_manager(robot)
        mgr.feed(self._name)


@dsl(hidden=True)
class StopWatchdog(Step):
    """Disarm and remove a named watchdog."""

    def __init__(self, name: str) -> None:
        super().__init__()
        if not isinstance(name, str) or not name:
            raise ValueError(f"Watchdog name must be a non-empty string: {name!r}")
        self._name = name

    def _generate_signature(self) -> str:
        return f"StopWatchdog(name={self._name!r})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        mgr = get_watchdog_manager(robot)
        mgr.disarm(self._name)


@dsl(tags=["control", "watchdog"])
def start_watchdog(
    name: str = DEFAULT_WATCHDOG_NAME,
    timeout: Union[float, int] = 5.0,
) -> StartWatchdog:
    """Arm a named watchdog that will kill the robot if not fed in time.

    Registers a keepalive watchdog with the robot's ``WatchdogManager``. If
    the watchdog is not fed via ``feed_watchdog`` before ``timeout`` seconds
    elapse, it expires and cancels the main-mission task — the same code
    path used by the global ``shutdown_in`` timer. The shutdown mission
    still runs, so motors are hard-stopped and the robot parks safely.

    Multiple watchdogs can run simultaneously by using distinct names. The
    default name covers the common single-watchdog case.

    Args:
        name: Unique identifier for this watchdog. Used by ``feed_watchdog``
            and ``stop_watchdog`` to address it. Defaults to ``"default"``.
        timeout: Seconds between feeds before expiry. Must be positive.

    Returns:
        A StartWatchdog step instance.

    Example::

        from raccoon.step import seq, start_watchdog, feed_watchdog, stop_watchdog
        from raccoon.step.motion import drive_forward

        seq(
            start_watchdog("scoring", timeout=5.0),
            drive_forward(30),
            feed_watchdog("scoring"),
            drive_forward(30),
            stop_watchdog("scoring"),
        )
    """
    return StartWatchdog(name, timeout)


@dsl(tags=["control", "watchdog"])
def feed_watchdog(name: str = DEFAULT_WATCHDOG_NAME) -> FeedWatchdog:
    """Reset a named watchdog's deadline to prevent expiry.

    Pushes the watchdog's deadline forward by its full timeout. Feeding an
    unarmed watchdog logs a warning but does not raise.

    Args:
        name: Identifier of the watchdog to feed. Defaults to ``"default"``.

    Returns:
        A FeedWatchdog step instance.

    Example::

        from raccoon.step import feed_watchdog

        feed_watchdog("scoring")
    """
    return FeedWatchdog(name)


@dsl(tags=["control", "watchdog"])
def stop_watchdog(name: str = DEFAULT_WATCHDOG_NAME) -> StopWatchdog:
    """Disarm a named watchdog so it no longer expires.

    Removes the watchdog from the manager. Stopping an unarmed watchdog
    logs a warning but does not raise — intended for cleanup paths where
    the watchdog may or may not still be active.

    Args:
        name: Identifier of the watchdog to disarm. Defaults to ``"default"``.

    Returns:
        A StopWatchdog step instance.

    Example::

        from raccoon.step import stop_watchdog

        stop_watchdog("scoring")
    """
    return StopWatchdog(name)
