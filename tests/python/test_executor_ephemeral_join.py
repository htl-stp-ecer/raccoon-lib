"""Runtime tests for PathExecutor ephemeral side-action joining.

Reproduces the bug where a parallel() side-effect branch (an EPHEMERAL
background SideAction) is launched but never awaited when the path ends without
a following segment transition to join at — the executor's ``finally`` then
cancels it before it finishes. This happens on optimized paths where
``to_absolute()`` / ``splinify()`` collapse the motion into inline side actions
(no real Segment left), or where a parallel sits at the tail of the path.

These are pure-Python tests — no simulator or C++ motion runtime is exercised,
because the reproduction paths contain only SideAction nodes (the executor's
leading side-action handling runs and returns before any motion segment).
"""

from __future__ import annotations

import asyncio
import importlib.util

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Lightweight fakes
# ---------------------------------------------------------------------------


class _TimedSideAction:
    """Stand-in for a SideAction's step: completes after ``delay`` seconds.

    Records whether it ran to completion vs. was cancelled mid-flight, so a
    test can tell a joined branch from a torn-down one.
    """

    def __init__(self, delay: float, resources: frozenset[str] = frozenset()) -> None:
        self._delay = delay
        self._resources = resources
        self.completed = False
        self.cancelled = False

    def collected_resources(self) -> frozenset[str]:
        # The executor inspects this on inline side actions to tell a collapsed
        # motion (holds "drive") from a real non-motion step.
        return self._resources

    async def run_step(self, robot) -> None:
        try:
            await asyncio.sleep(self._delay)
            self.completed = True
        except asyncio.CancelledError:
            self.cancelled = True
            raise


class _FakeDrive:
    def __init__(self) -> None:
        self.hard_stops = 0

    def hard_stop(self) -> None:
        self.hard_stops += 1


class _FakeRobot:
    def __init__(self) -> None:
        self.drive = _FakeDrive()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@requires_libstp
def test_ephemeral_branch_joined_when_path_is_only_side_actions():
    """A parallel branch concurrent with collapsed (inline) motion is joined.

    Mirrors ``optimize(...).to_absolute()`` on a mostly-linear mission: the
    spine motion becomes ONE inline ``GotoWaypoints`` side action and the
    parallel branch is an ephemeral background SideAction emitted before it.
    There is no real Segment, so the path runs entirely in the leading
    side-action pass and returns — the only chance to honour parallel()'s
    await-all is the path-end join. Before the fix the branch was cancelled.
    """
    from raccoon.step.motion.path.executor import PathExecutor
    from raccoon.step.motion.path.ir import SideAction

    branch = _TimedSideAction(0.05)  # parallel arm action — outlives the drive
    # Collapsed GotoWaypoints holds the drive resource, so the executor runs it
    # CONCURRENTLY with the branch and joins the branch at path end.
    inline_drive = _TimedSideAction(0.01, resources=frozenset({"drive"}))

    nodes = [
        SideAction(step=branch, is_background=True, ephemeral=True),
        SideAction(step=inline_drive, is_background=False),
    ]

    asyncio.run(PathExecutor(nodes=nodes, deferred=[]).run(_FakeRobot()))

    assert inline_drive.completed, "inline (collapsed motion) side action must run"
    assert branch.completed, "ephemeral parallel branch must be joined, not cancelled"
    assert not branch.cancelled


@requires_libstp
def test_lone_ephemeral_branch_is_awaited():
    """A path of a single ephemeral side action runs it to completion."""
    from raccoon.step.motion.path.executor import PathExecutor
    from raccoon.step.motion.path.ir import SideAction

    branch = _TimedSideAction(0.02)
    nodes = [SideAction(step=branch, is_background=True, ephemeral=True)]

    asyncio.run(PathExecutor(nodes=nodes, deferred=[]).run(_FakeRobot()))

    assert branch.completed
    assert not branch.cancelled


@requires_libstp
def test_plain_background_stays_fire_and_forget():
    """A non-ephemeral background() is still cancelled at path end (by design).

    Only ephemeral parallel branches are join-guaranteed; plain ``background()``
    keeps its documented fire-and-forget semantics, so a long-running one is
    torn down when the path ends rather than blocking it.
    """
    from raccoon.step.motion.path.executor import PathExecutor
    from raccoon.step.motion.path.ir import SideAction

    bg = _TimedSideAction(10.0)  # would block the path if (wrongly) awaited
    # A quick inline action after it yields the event loop so the bg task
    # actually starts (reaches its await) before the path ends — otherwise it
    # would be cancelled before ever running and never observe the teardown.
    inline = _TimedSideAction(0.01)
    nodes = [
        SideAction(step=bg, is_background=True, ephemeral=False),
        SideAction(step=inline, is_background=False),
    ]

    asyncio.run(PathExecutor(nodes=nodes, deferred=[]).run(_FakeRobot()))

    assert inline.completed
    assert not bg.completed, "plain background() must not block the path to completion"
    assert bg.cancelled, "plain background() is torn down (fire-and-forget) at path end"
