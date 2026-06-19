"""Lowering barrier tests — the optimizer must NEVER crash on a step that
lowers to no segments.

Contract: any step whose ``lower_to_segments()`` yields nothing becomes an
inline ``SideAction`` (opaque barrier) instead of raising. Drive-resource
steps additionally emit a one-time WARNING (they're a yellow flag — a drive
step that can't describe its motion runs unoptimized). ``TimeoutOr`` is a
legitimate opaque composite and is treated as a clean barrier (no warning).
"""

from __future__ import annotations

import importlib.util
import logging

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon (libstp) not importable",
)


@requires_libstp
class TestLoweringBarrier:
    def test_unsupported_drive_step_becomes_barrier_not_raise(self, caplog):
        """A drive-resource step that returns None from lower_to_segments is
        run unoptimized as a single inline SideAction — no exception."""
        from raccoon.step.base import Step
        from raccoon.step.motion.path.ir import SideAction
        from raccoon.step.motion.path.passes import flatten_steps

        class UnsupportedDriveStep(Step):
            def collected_resources(self):
                return frozenset({"drive"})

            def _generate_signature(self):
                return "UnsupportedDriveStep()"

            async def _execute_step(self, robot):
                pass

        step = UnsupportedDriveStep()

        with caplog.at_level(logging.WARNING):
            nodes, deferred = flatten_steps([step])

        assert len(nodes) == 1
        assert isinstance(nodes[0], SideAction)
        assert nodes[0].is_background is False
        assert nodes[0].step is step
        assert deferred == []
        # Drive-resource steps are a yellow flag → a warning is emitted.
        assert any(
            "UnsupportedDriveStep" in rec.getMessage()
            for rec in caplog.records
            if rec.levelno == logging.WARNING
        )

    def test_timeout_or_becomes_barrier_no_warning(self, caplog):
        """timeout_or() is a legitimate opaque composite — inline SideAction,
        no exception and no 'unsupported drive step' warning."""
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.path.ir import SideAction
        from raccoon.step.motion.path.passes import flatten_steps
        from raccoon.step.timeout_or import timeout_or

        step = timeout_or(
            drive_forward(cm=30.0),
            seconds=3.0,
            fallback=drive_forward(cm=5.0),
        )

        with caplog.at_level(logging.WARNING):
            nodes, deferred = flatten_steps([step])

        assert len(nodes) == 1
        assert isinstance(nodes[0], SideAction)
        assert nodes[0].is_background is False
        assert deferred == []
        # It's not negligence, so no unsupported-drive warning.
        assert not any(
            "does not lower to motion segments" in rec.getMessage() for rec in caplog.records
        )

    def test_normal_motion_still_lowers_to_segment(self):
        """Regression: a normal drive leg still produces a Segment."""
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.path.ir import Segment
        from raccoon.step.motion.path.passes import flatten_steps

        nodes, deferred = flatten_steps([drive_forward(cm=20.0)])

        assert len(nodes) == 1
        assert isinstance(nodes[0], Segment)
        assert deferred == []

    def test_non_drive_step_still_becomes_side_action(self):
        """A non-drive step (wait) becomes an inline SideAction (unchanged)."""
        from raccoon.step.motion.path.ir import SideAction
        from raccoon.step.motion.path.passes import flatten_steps
        from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

        nodes, deferred = flatten_steps([wait_for_seconds(seconds=1.0)])

        assert len(nodes) == 1
        assert isinstance(nodes[0], SideAction)
        assert nodes[0].is_background is False
        assert deferred == []
