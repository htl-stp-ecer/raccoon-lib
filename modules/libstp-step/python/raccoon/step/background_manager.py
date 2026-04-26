"""Background step manager — tracks running background tasks and handles preemption.

When a foreground step requires a resource held by a background step, the
background step is cancelled (preempted) with a warning instead of raising
``ResourceConflictError``.  The manager also logs when background steps
complete, fail, or get cancelled.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

from .resource import _resources_overlap

logger = logging.getLogger(__name__)


@dataclass
class _BGEntry:
    task: asyncio.Task[None]
    label: str
    name: str | None
    resources: frozenset[str]


class BackgroundManager:
    """Track background step tasks and handle resource preemption.

    Attached to the robot instance as ``robot._background_manager``.
    """

    def __init__(self) -> None:
        self._entries: dict[int, _BGEntry] = {}
        self._named: dict[str, _BGEntry] = {}

    def register(
        self,
        task: asyncio.Task[None],
        label: str,
        name: str | None,
        resources: frozenset[str],
    ) -> None:
        """Register a background task for tracking and preemption."""
        entry = _BGEntry(task=task, label=label, name=name, resources=resources)
        self._entries[id(task)] = entry
        if name is not None:
            if name in self._named and not self._named[name].task.done():
                logger.warning(
                    "Background name '%s' reused — previous task still running",
                    name,
                )
            self._named[name] = entry
        task.add_done_callback(self._on_done)

    def _on_done(self, task: asyncio.Task[None]) -> None:
        entry = self._entries.pop(id(task), None)
        if entry is None:
            return
        # Only remove named entry if it still points to this task — a later
        # registration with the same name may have already replaced it.
        if entry.name is not None and self._named.get(entry.name) is entry:
            del self._named[entry.name]
        if task.cancelled():
            logger.debug("Background step '%s' cancelled", entry.label)
        elif task.exception():
            logger.error("Background step '%s' failed: %s", entry.label, task.exception())
        else:
            logger.info("Background step '%s' completed", entry.label)

    async def preempt_conflicts(self, resources: frozenset[str], holder_label: str) -> None:
        """Cancel background tasks whose resources overlap with *resources*.

        After cancellation, awaits cleanup so released resources are
        available for immediate acquisition.
        """
        to_cancel: list[_BGEntry] = []
        for entry in list(self._entries.values()):
            if entry.task.done():
                continue
            overlaps = _resources_overlap(resources, entry.resources)
            if overlaps:
                logger.warning(
                    "Foreground '%s' preempting background '%s' " "— conflicting: %s",
                    holder_label,
                    entry.label,
                    ", ".join(overlaps),
                )
                to_cancel.append(entry)

        for entry in to_cancel:
            entry.task.cancel()

        if to_cancel:
            await asyncio.gather(
                *[entry.task for entry in to_cancel],
                return_exceptions=True,
            )

    async def wait_all(self) -> None:
        """Wait for all running background tasks to complete."""
        tasks = [e.task for e in list(self._entries.values()) if not e.task.done()]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def wait_for_name(self, name: str) -> None:
        """Wait for a specific named background task to complete.

        Either outcome (clean exit, cancellation, or exception inside the
        task) is acceptable here — the caller is asking "tell me when it's
        done", not "tell me how it ended". We split the two ``except``
        clauses on purpose: ``CancelledError`` is a ``BaseException`` in
        Python 3.8+, so the previous ``except (CancelledError, Exception)``
        tuple was misleading. Task-internal failures get a debug log so a
        diagnostic trail exists; cancellation does not, because it is the
        expected outcome of mission shutdown.
        """
        entry = self._named.get(name)
        if entry is None:
            logger.debug(
                "No background step named '%s' (already done or never started)",
                name,
            )
            return
        try:
            await entry.task
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            logger.debug("Background step '%s' ended with %r", name, exc)

    async def cancel_all(self) -> None:
        """Cancel all running background tasks and await cleanup.

        Called at mission boundaries and during shutdown to prevent
        orphaned tasks from leaking or triggering asyncio warnings.
        """
        active = [e for e in list(self._entries.values()) if not e.task.done()]
        if not active:
            return

        for entry in active:
            logger.debug("Cancelling orphaned background step '%s'", entry.label)
            entry.task.cancel()

        await asyncio.gather(
            *[entry.task for entry in active],
            return_exceptions=True,
        )

    @property
    def active_count(self) -> int:
        """Number of background tasks still running."""
        return sum(1 for e in self._entries.values() if not e.task.done())


def get_background_manager(robot: object) -> BackgroundManager:
    """Get or create the ``BackgroundManager`` attached to *robot*."""
    mgr = getattr(robot, "_background_manager", None)
    if mgr is None:
        mgr = BackgroundManager()
        robot._background_manager = mgr  # type: ignore[attr-defined]
    return mgr
