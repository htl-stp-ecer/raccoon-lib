"""Hardware resource tracking and conflict detection for parallel step execution.

Every step declares which hardware resources it requires exclusive access to via
``required_resources()``.  Composite steps (Parallel, DoWhileActive) validate at
construction time that no two concurrent branches claim the same resource.  A
lightweight runtime ``ResourceManager`` acts as a safety net for dynamically
constructed steps (e.g. ``Defer``) that cannot be checked statically.

Resource identifiers are plain strings with the format ``"<type>"`` or
``"<type>:<qualifier>"``:

    "drive"       — chassis drive system
    "motor:0"     — motor on port 0
    "servo:3"     — servo on port 3
    "servo:*"     — all servo ports (used by FullyDisableServos)
"""

from __future__ import annotations

from collections.abc import Sequence


class ResourceConflictError(Exception):
    """Raised when two concurrent steps require the same hardware resource."""

    def __init__(self, resource: str, holder: str, requester: str) -> None:
        self.resource = resource
        self.holder = holder
        self.requester = requester
        super().__init__(
            f"Resource conflict on '{resource}': already held by "
            f"{holder} when {requester} tried to acquire it"
        )


def _resources_overlap(a: frozenset[str], b: frozenset[str]) -> list[str]:
    """Return the list of resource IDs that conflict between *a* and *b*.

    Handles wildcard resources (e.g. ``"servo:*"`` conflicts with any
    ``"servo:<n>"``).
    """
    conflicts: list[str] = []

    # Direct overlap
    direct = a & b
    if direct:
        conflicts.extend(sorted(direct))

    # Wildcard expansion: "servo:*" in one set conflicts with "servo:N" in the other
    for res in a:
        if res.endswith(":*"):
            prefix = res.rsplit(":", 1)[0] + ":"
            for other in b:
                if other.startswith(prefix) and other != res and other not in conflicts:
                    conflicts.append(other)

    for res in b:
        if res.endswith(":*"):
            prefix = res.rsplit(":", 1)[0] + ":"
            for other in a:
                if other.startswith(prefix) and other != res and other not in conflicts:
                    conflicts.append(other)

    return conflicts


def validate_no_overlap(
    branches: Sequence["_HasResources"],
    context: str = "Parallel",
) -> None:
    """Check pairwise that no two branches claim the same resource.

    Args:
        branches: Sequence of objects with a ``required_resources()`` method.
        context: Name used in the error message (e.g. ``"Parallel"``
            or ``"DoWhileActive"``).

    Raises:
        ResourceConflictError: If any two branches share a resource.
    """
    resource_sets: list[frozenset[str]] = [b.collected_resources() for b in branches]

    for i in range(len(resource_sets)):
        for j in range(i + 1, len(resource_sets)):
            conflicts = _resources_overlap(resource_sets[i], resource_sets[j])
            if conflicts:
                label_i = _branch_label(branches[i], i)
                label_j = _branch_label(branches[j], j)
                raise ResourceConflictError(
                    resource=", ".join(conflicts),
                    holder=f"{context} branch {i} ({label_i})",
                    requester=f"{context} branch {j} ({label_j})",
                )


def _branch_label(step: "_HasResources", _index: int) -> str:
    """Best-effort human-readable label for an error message."""
    cls_name = type(step).__name__
    sig = getattr(step, "_generate_signature", None)
    if sig is not None:
        try:
            return sig()
        except Exception:
            pass
    return cls_name


# ---------------------------------------------------------------------------
# Runtime resource manager (safety net for Defer / Run)
# ---------------------------------------------------------------------------


class ResourceManager:
    """Non-blocking, fail-fast resource lock attached to the robot instance.

    Since all steps run in a single asyncio event loop (cooperative
    multitasking), no real lock is needed — a dict suffices.

    Acquisition is **re-entrant within the same asyncio task**: a composite
    motion step (``smooth_path`` / ``optimize``) acquires ``"drive"`` and then,
    inline in the SAME task, runs sub-steps that also need ``"drive"`` (e.g. a
    ``GotoWaypoints`` emitted by ``to_absolute``). Those nested acquires are
    sequential, not concurrent, so they nest via a depth counter rather than
    conflicting. Concurrent claims from a DIFFERENT task — parallel branches
    and ``background()`` steps run under ``asyncio.create_task`` — still
    conflict, which is the property the guard exists to enforce.
    """

    def __init__(self) -> None:
        # resource_id → (holder_label, owner_task, depth)
        self._held: dict[str, tuple[str, object | None, int]] = {}

    @staticmethod
    def _current_task() -> object | None:
        import asyncio

        try:
            return asyncio.current_task()
        except RuntimeError:  # no running loop
            return None

    def _conflict(self, res: str, task: object | None) -> tuple[str, str] | None:
        """Return ``(held_res, holder)`` if *res* clashes with a DIFFERENT task.

        A resource held by the SAME task is re-entrant — no conflict.
        """

        def held_by_other(key: str) -> tuple[str, str] | None:
            entry = self._held.get(key)
            if entry is None:
                return None
            holder, owner, _depth = entry
            if owner is task and task is not None:
                return None  # same task — re-entrant
            return (key, holder)

        # Direct conflict
        hit = held_by_other(res)
        if hit is not None:
            return hit
        # This res is a wildcard covering held "prefix:N"
        if res.endswith(":*"):
            prefix = res.rsplit(":", 1)[0] + ":"
            for held_res in self._held:
                if held_res.startswith(prefix):
                    hit = held_by_other(held_res)
                    if hit is not None:
                        return hit
        else:
            # A held wildcard covers this res
            parts = res.rsplit(":", 1)
            if len(parts) == 2:
                hit = held_by_other(parts[0] + ":*")
                if hit is not None:
                    return (res, hit[1])
        return None

    def acquire(self, resources: frozenset[str], holder: str) -> None:
        """Claim *resources* for *holder*.  Raises on cross-task conflict."""
        task = self._current_task()
        for res in resources:
            conflict = self._conflict(res, task)
            if conflict is not None:
                held_res, held_holder = conflict
                raise ResourceConflictError(held_res, held_holder, holder)

        # All clear — register (nesting within the same task bumps depth).
        for res in resources:
            entry = self._held.get(res)
            if entry is not None and entry[1] is task and task is not None:
                self._held[res] = (entry[0], task, entry[2] + 1)
            else:
                self._held[res] = (holder, task, 1)

    def release(self, resources: frozenset[str]) -> None:
        """Release previously acquired resources (depth-aware)."""
        for res in resources:
            entry = self._held.get(res)
            if entry is None:
                continue
            holder, owner, depth = entry
            if depth > 1:
                self._held[res] = (holder, owner, depth - 1)
            else:
                self._held.pop(res, None)


def get_resource_manager(robot: object) -> ResourceManager:
    """Get or create the ``ResourceManager`` attached to *robot*."""
    mgr = getattr(robot, "_resource_manager", None)
    if mgr is None:
        mgr = ResourceManager()
        robot._resource_manager = mgr  # type: ignore[attr-defined]
    return mgr


# Type alias used only for the validate_no_overlap signature
class _HasResources:
    def collected_resources(self) -> frozenset[str]: ...
