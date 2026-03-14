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

from typing import List, Sequence


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
    resource_sets: List[frozenset[str]] = [
        b.collected_resources() for b in branches
    ]

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


def _branch_label(step: "_HasResources", index: int) -> str:
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
    multitasking), no real lock is needed — a simple dict suffices.
    """

    def __init__(self) -> None:
        self._held: dict[str, str] = {}  # resource_id → holder label

    def acquire(self, resources: frozenset[str], holder: str) -> None:
        """Claim *resources* for *holder*.  Raises on conflict."""
        for res in resources:
            # Check direct conflict
            if res in self._held:
                raise ResourceConflictError(res, self._held[res], holder)
            # Check wildcard conflict
            if res.endswith(":*"):
                prefix = res.rsplit(":", 1)[0] + ":"
                for held_res, held_holder in self._held.items():
                    if held_res.startswith(prefix):
                        raise ResourceConflictError(
                            held_res, held_holder, holder
                        )
            else:
                # Check if a wildcard is already held that covers this resource
                parts = res.rsplit(":", 1)
                if len(parts) == 2:
                    wildcard = parts[0] + ":*"
                    if wildcard in self._held:
                        raise ResourceConflictError(
                            res, self._held[wildcard], holder
                        )

        # All clear — register
        for res in resources:
            self._held[res] = holder

    def release(self, resources: frozenset[str]) -> None:
        """Release previously acquired resources."""
        for res in resources:
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
