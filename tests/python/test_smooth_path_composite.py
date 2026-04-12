"""Construction-time tests for smooth_path() composite step support.

Validates that smooth_path correctly flattens nested seq(), parallel(),
background(), Run(), and non-drive steps, and rejects invalid combinations.
These are pure-Python tests — no simulator or C++ runtime required beyond
the raccoon package being installed.
"""
from __future__ import annotations

import asyncio

import pytest


def _libstp_available():
    try:
        import raccoon  # noqa: F401
        return True
    except ImportError:
        return False


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Helpers: lightweight step stubs
# ---------------------------------------------------------------------------

def _make_stubs():
    """Import Step and build minimal stubs for testing flattening logic."""
    from raccoon.step.base import Step

    class ServoStub(Step):
        """Non-drive step (servo-like)."""

        def __init__(self, port: int = 0):
            super().__init__()
            self._port = port

        def required_resources(self):
            return frozenset({f"servo:{self._port}"})

        def _generate_signature(self):
            return f"ServoStub(port={self._port})"

        async def _execute_step(self, robot):
            pass

    class UnsupportedDriveStep(Step):
        """Uses drive resource but isn't a supported motion type."""

        def required_resources(self):
            return frozenset({"drive"})

        def _generate_signature(self):
            return "UnsupportedDriveStep()"

        async def _execute_step(self, robot):
            pass

    return ServoStub, UnsupportedDriveStep


# ---------------------------------------------------------------------------
# Flattening: nested seq()
# ---------------------------------------------------------------------------


@requires_libstp
class TestNestedSeq:
    def test_seq_is_flattened(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.sequential import seq

        inner = seq([drive_forward(cm=10.0), drive_forward(cm=20.0)])
        sp = SmoothPath([inner, drive_forward(cm=30.0)])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        assert len(segments) == 3, f"expected 3 segments, got {len(segments)}"
        # Distances: 10cm, 20cm, 30cm (converted to meters with sign)
        distances_cm = [abs(s.distance_m) * 100 for s in segments]
        assert distances_cm == pytest.approx([10.0, 20.0, 30.0], abs=0.1)

    def test_deeply_nested_seq(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.sequential import seq

        inner = seq([seq([drive_forward(cm=5.0), drive_forward(cm=5.0)])])
        sp = SmoothPath([inner])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        assert len(segments) == 2


# ---------------------------------------------------------------------------
# Flattening: parallel()
# ---------------------------------------------------------------------------


@requires_libstp
class TestParallelFlattening:
    def test_parallel_with_motion_spine_and_side_effect(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.parallel import parallel

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            parallel(
                [drive_forward(cm=10.0), drive_forward(cm=20.0)],
                [ServoStub()],
            ),
        ])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        side_actions = [n for n in sp._nodes if isinstance(n, _SideAction)]
        assert len(segments) == 2, "motion spine should have 2 segments"
        assert len(side_actions) == 1, "should have 1 side-action branch"
        assert side_actions[0].is_background is True

    def test_parallel_no_drive_branch_becomes_side_action(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.parallel import parallel

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=10.0),
            parallel(
                [ServoStub(port=0)],
                [ServoStub(port=1)],
            ),
            drive_forward(cm=20.0),
        ])

        side_actions = [n for n in sp._nodes if isinstance(n, _SideAction)]
        assert len(side_actions) == 1
        assert side_actions[0].is_background is False  # inline, no drive spine

    def test_side_actions_placed_before_spine_segments(self):
        """Side-effect branches should appear before the spine segments."""
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.parallel import parallel

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            parallel(
                [drive_forward(cm=10.0)],
                [ServoStub()],
            ),
        ])

        # First node should be the side action, second the segment
        assert isinstance(sp._nodes[0], _SideAction)
        assert isinstance(sp._nodes[1], _Segment)


# ---------------------------------------------------------------------------
# Flattening: background()
# ---------------------------------------------------------------------------


@requires_libstp
class TestBackgroundFlattening:
    def test_background_becomes_side_action(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.logic.background import background

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=10.0),
            background(ServoStub()),
            drive_forward(cm=20.0),
        ])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        side_actions = [n for n in sp._nodes if isinstance(n, _SideAction)]
        assert len(segments) == 2
        assert len(side_actions) == 1
        assert side_actions[0].is_background is True


# ---------------------------------------------------------------------------
# Flattening: Run()
# ---------------------------------------------------------------------------


@requires_libstp
class TestRunFlattening:
    def test_run_becomes_inline_side_action(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.logic.defer import Run

        sp = SmoothPath([
            drive_forward(cm=10.0),
            Run(lambda robot: None),
            drive_forward(cm=20.0),
        ])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        side_actions = [n for n in sp._nodes if isinstance(n, _SideAction)]
        assert len(segments) == 2
        assert len(side_actions) == 1
        assert side_actions[0].is_background is False


# ---------------------------------------------------------------------------
# Flattening: non-drive step (inline side action)
# ---------------------------------------------------------------------------


@requires_libstp
class TestNonDriveStepFlattening:
    def test_non_drive_step_becomes_inline(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=10.0),
            ServoStub(),
            drive_forward(cm=20.0),
        ])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        side_actions = [n for n in sp._nodes if isinstance(n, _SideAction)]
        assert len(segments) == 2
        assert len(side_actions) == 1
        assert side_actions[0].is_background is False


# ---------------------------------------------------------------------------
# Validation: construction-time errors
# ---------------------------------------------------------------------------


@requires_libstp
class TestConstructionTimeValidation:
    def test_empty_raises(self):
        from raccoon.step.motion.smooth_path import SmoothPath

        with pytest.raises(ValueError, match="requires at least one step"):
            SmoothPath([])

    def test_no_motion_steps_raises(self):
        from raccoon.step.motion.smooth_path import SmoothPath

        ServoStub, _ = _make_stubs()

        with pytest.raises(ValueError, match="requires at least one motion step"):
            SmoothPath([ServoStub()])

    def test_unsupported_drive_step_raises(self):
        from raccoon.step.motion.smooth_path import SmoothPath

        _, UnsupportedDriveStep = _make_stubs()

        with pytest.raises(TypeError, match="does not support UnsupportedDriveStep"):
            SmoothPath([UnsupportedDriveStep()])

    def test_deferred_only_does_not_raise(self):
        """A path with only Defer should be accepted at construction."""
        from raccoon.step.motion.smooth_path import SmoothPath
        from raccoon.step.logic.defer import Defer
        from raccoon.step.motion.drive_dsl import drive_forward

        sp = SmoothPath([Defer(lambda robot: drive_forward(cm=10.0))])
        # Should not raise — validation deferred to runtime
        assert len(sp._deferred) == 1


# ---------------------------------------------------------------------------
# Signature generation
# ---------------------------------------------------------------------------


@requires_libstp
class TestSignature:
    def test_signature_includes_side_actions(self):
        from raccoon.step.motion.smooth_path import SmoothPath
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.logic.background import background

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=10.0),
            background(ServoStub()),
            drive_forward(cm=20.0),
        ])

        sig = sp._generate_signature()
        assert "drive(" in sig
        assert "bg)" in sig  # background marker

    def test_signature_with_inline(self):
        from raccoon.step.motion.smooth_path import SmoothPath
        from raccoon.step.motion.drive_dsl import drive_forward

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=10.0),
            ServoStub(),
            drive_forward(cm=20.0),
        ])

        sig = sp._generate_signature()
        assert "inline)" in sig


# ---------------------------------------------------------------------------
# collected_resources includes side actions
# ---------------------------------------------------------------------------


@requires_libstp
class TestCollectedResources:
    def test_includes_drive_and_side_action_resources(self):
        from raccoon.step.motion.smooth_path import SmoothPath
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.logic.background import background

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=10.0),
            background(ServoStub()),
        ])

        resources = sp.collected_resources()
        assert "drive" in resources
        assert "servo:0" in resources


# ---------------------------------------------------------------------------
# Mixed composite: seq inside parallel inside smooth_path
# ---------------------------------------------------------------------------


@requires_libstp
class TestMixedComposite:
    def test_seq_inside_parallel(self):
        from raccoon.step.motion.smooth_path import SmoothPath, _Segment, _SideAction
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.sequential import seq
        from raccoon.step.parallel import parallel

        ServoStub, _ = _make_stubs()

        sp = SmoothPath([
            drive_forward(cm=5.0),
            parallel(
                seq([drive_forward(cm=10.0), drive_forward(cm=15.0)]),
                [ServoStub()],
            ),
            drive_forward(cm=20.0),
        ])

        segments = [n for n in sp._nodes if isinstance(n, _Segment)]
        side_actions = [n for n in sp._nodes if isinstance(n, _SideAction)]
        assert len(segments) == 4  # 5 + 10 + 15 + 20
        assert len(side_actions) == 1  # servo side-effect
