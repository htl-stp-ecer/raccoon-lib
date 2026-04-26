"""Watchdog manager — deadlines that cancel the main-mission task on expiry.

Two usage patterns share one manager:

- **Mission deadlines** — armed automatically by the robot runner from a
  ``Mission.time_budget`` class attribute. Never fed; expiry fires when the
  budget elapses.
- **Keepalive watchdogs** — armed/fed/disarmed imperatively by user code
  via the ``start_watchdog`` / ``feed_watchdog`` / ``stop_watchdog`` steps.
  Must be fed before their timeout or expiry fires.

On expiry, the manager cancels the registered *main-mission task*. This
routes through the existing shutdown path in ``_run_missions``: background
tasks drain, the shutdown mission runs, then the process exits.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)


class WatchdogExpiredError(Exception):
    """Raised on the main-mission task when a watchdog expires."""


@dataclass
class _WDEntry:
    name: str
    timeout: float
    deadline: float
    task: asyncio.Task[None]
    source: str  # "mission" or "user"


class WatchdogManager:
    """Track armed watchdogs and cancel the main-mission task on expiry.

    Attached to the robot instance as ``robot._watchdog_manager``. Wire the
    main-mission task via ``attach_main_task`` before arming anything so the
    manager knows what to cancel.
    """

    def __init__(self) -> None:
        self._entries: dict[str, _WDEntry] = {}
        self._main_task: asyncio.Task[None] | None = None
        self._expired_name: str | None = None

    def attach_main_task(self, task: asyncio.Task[None]) -> None:
        """Register the task that should be cancelled when a watchdog fires."""
        self._main_task = task
        self._expired_name = None

    def detach_main_task(self) -> None:
        """Clear the main-task reference after main missions finish."""
        self._main_task = None

    @property
    def expired_name(self) -> str | None:
        """Name of the watchdog that fired, or ``None`` if no expiry occurred."""
        return self._expired_name

    def arm(self, name: str, timeout: float, source: str = "user") -> None:
        """Arm a watchdog. Replaces any existing entry with the same name."""
        if timeout <= 0:
            msg = f"Watchdog timeout must be positive: {timeout}"
            raise ValueError(msg)

        existing = self._entries.get(name)
        if existing is not None:
            logger.warning(
                "Watchdog '%s' re-armed while already active — previous entry cancelled",
                name,
            )
            existing.task.cancel()

        loop = asyncio.get_event_loop()
        deadline = loop.time() + timeout
        task = asyncio.create_task(self._expiry_timer(name))
        self._entries[name] = _WDEntry(
            name=name, timeout=timeout, deadline=deadline, task=task, source=source
        )
        logger.debug("Armed watchdog '%s' (source=%s, timeout=%.2fs)", name, source, timeout)

    def feed(self, name: str) -> None:
        """Push the named watchdog's deadline out by its full timeout."""
        entry = self._entries.get(name)
        if entry is None:
            logger.warning("Cannot feed watchdog '%s' — not armed", name)
            return

        loop = asyncio.get_event_loop()
        entry.deadline = loop.time() + entry.timeout
        logger.debug("Fed watchdog '%s' — next deadline in %.2fs", name, entry.timeout)

    def disarm(self, name: str, *, missing_ok: bool = False) -> None:
        """Cancel and remove the named watchdog."""
        entry = self._entries.pop(name, None)
        if entry is None:
            if not missing_ok:
                logger.warning("Cannot disarm watchdog '%s' — not armed", name)
            return
        entry.task.cancel()
        logger.debug("Disarmed watchdog '%s'", name)

    def active_names(self, source: str | None = None) -> list[str]:
        """Return names of currently armed watchdogs, optionally filtered by source."""
        if source is None:
            return list(self._entries.keys())
        return [n for n, e in self._entries.items() if e.source == source]

    async def cancel_all(self) -> None:
        """Cancel every armed watchdog and await cleanup."""
        entries = list(self._entries.values())
        self._entries.clear()
        for entry in entries:
            entry.task.cancel()
        if entries:
            await asyncio.gather(*[e.task for e in entries], return_exceptions=True)

    async def _expiry_timer(self, name: str) -> None:
        """Sleep until the entry's deadline, then fire expiry."""
        try:
            while True:
                entry = self._entries.get(name)
                if entry is None:
                    return
                loop = asyncio.get_event_loop()
                remaining = entry.deadline - loop.time()
                if remaining <= 0:
                    self._fire_expiry(entry)
                    return
                await asyncio.sleep(remaining)
        except asyncio.CancelledError:
            return

    def _fire_expiry(self, entry: _WDEntry) -> None:
        """Record expiry and cancel the main-mission task."""
        self._entries.pop(entry.name, None)
        self._expired_name = entry.name
        logger.error(
            "Watchdog '%s' (source=%s) expired after %.2fs — killing robot",
            entry.name,
            entry.source,
            entry.timeout,
        )
        main = self._main_task
        if main is None:
            logger.error(
                "Watchdog '%s' expired but no main-mission task attached — cannot cancel",
                entry.name,
            )
            return
        if not main.done():
            main.cancel()


def get_watchdog_manager(robot: object) -> WatchdogManager:
    """Get or create the ``WatchdogManager`` attached to *robot*."""
    mgr = getattr(robot, "_watchdog_manager", None)
    if mgr is None:
        mgr = WatchdogManager()
        robot._watchdog_manager = mgr  # type: ignore[attr-defined]
    return mgr
