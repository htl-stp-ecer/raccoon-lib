"""Unit tests for the fluent ``optimize([...])`` path-optimizer builder.

Covers the compile-time pass state machine, the chain methods (merge,
cut_corners, apply), ``explain()`` diagnostics, and the drop-in (no-pass)
shape.  No simulator or C++ runtime is required beyond the raccoon package.
"""

from __future__ import annotations

import importlib.util

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Imports — deferred behind requires_libstp so collection doesn't fail
# ---------------------------------------------------------------------------


def _imports():
    from raccoon.step.base import Step
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.optimize import (
        Optimizer,
        PathBuildError,
        Representation,
        optimize,
    )
    from raccoon.step.motion.path.passes import CornerCutPass, MergePass, flatten_steps
    from raccoon.step.motion.turn_dsl import turn_right

    return {
        "Step": Step,
        "Segment": Segment,
        "Optimizer": Optimizer,
        "PathBuildError": PathBuildError,
        "Representation": Representation,
        "optimize": optimize,
        "CornerCutPass": CornerCutPass,
        "MergePass": MergePass,
        "flatten_steps": flatten_steps,
        "drive_forward": drive_forward,
        "turn_right": turn_right,
    }


def _compiled_nodes(opt):
    """Compile an Optimizer's plan through the same path the executor uses."""
    from raccoon.step.motion.path.compiler import PathCompiler

    return PathCompiler(opt._passes).compile(opt._raw_steps).nodes


# ---------------------------------------------------------------------------
# Build + compile
# ---------------------------------------------------------------------------


class TestBuildAndCompile:
    @requires_libstp
    def test_linear_turn_linear_compiles(self):
        imp = _imports()
        opt = imp["optimize"](
            [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](30)]
        )
        nodes = _compiled_nodes(opt)
        kinds = [n.kind for n in nodes if isinstance(n, imp["Segment"])]
        assert kinds == ["linear", "turn", "linear"]

    @requires_libstp
    def test_explain_mentions_linear(self):
        imp = _imports()
        opt = imp["optimize"](
            [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](30)]
        )
        text = opt.explain()
        assert "linear" in text


# ---------------------------------------------------------------------------
# merge — now ALWAYS-ON (no .merge() method); _compiled_nodes drives the
# effective pipeline (decompose + merge prepended) since it compiles
# opt._effective_passes() below.
# ---------------------------------------------------------------------------


def _effective_compiled_nodes(opt):
    """Compile through the effective (always-on prepended) pipeline."""
    from raccoon.step.motion.path.compiler import PathCompiler

    return PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes


class TestMerge:
    @requires_libstp
    def test_merge_is_always_on(self):
        """Two adjacent drives merge with NO .merge() call (merge is always-on)."""
        imp = _imports()
        opt = imp["optimize"]([imp["drive_forward"](20), imp["drive_forward"](30)])
        # No user passes were chained.
        assert opt._passes == []
        # But the effective pipeline prepends decompose + merge.
        names = [p.name for p in opt._effective_passes()]
        assert names[:2] == ["decompose", "merge"]

    @requires_libstp
    def test_merge_collapses_two_adjacent_drives(self):
        imp = _imports()
        opt = imp["optimize"]([imp["drive_forward"](20), imp["drive_forward"](30)])
        after = [n for n in _effective_compiled_nodes(opt) if isinstance(n, imp["Segment"])]
        # ONE merged linear (0.50 m) even though no .merge() was called.
        assert len(after) == 1
        assert abs(after[0].distance_m - 0.5) < 1e-6


# ---------------------------------------------------------------------------
# cut_corners()
# ---------------------------------------------------------------------------


class TestCutCorners:
    @requires_libstp
    def test_cut_corners_appends_pass_with_converted_radius(self):
        imp = _imports()
        opt = imp["optimize"](
            [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](40)]
        ).cut_corners(5)
        assert len(opt._passes) == 1
        assert isinstance(opt._passes[0], imp["CornerCutPass"])
        assert abs(opt._passes[0].cut_m - 0.05) < 1e-9

    @requires_libstp
    def test_cut_corners_inserts_arc(self):
        imp = _imports()
        opt = imp["optimize"](
            [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](40)]
        ).cut_corners(5)
        kinds = [n.kind for n in _compiled_nodes(opt) if isinstance(n, imp["Segment"])]
        assert kinds == ["linear", "arc", "linear"]


# ---------------------------------------------------------------------------
# _add state machine
# ---------------------------------------------------------------------------


class _DummyPass:
    name = "dummy"

    def __init__(self, *, requires=None, produces=None, terminal=False):
        if requires is not None:
            self.requires = requires
        if produces is not None:
            self.produces = produces
        self.terminal = terminal

    def run(self, nodes):
        return nodes


class TestStateMachine:
    @requires_libstp
    def test_absolute_requiring_pass_on_relative_stream_raises(self):
        imp = _imports()
        dummy = _DummyPass(requires=imp["Representation"].ABSOLUTE)
        opt = imp["optimize"]([imp["drive_forward"](20)])
        with pytest.raises(imp["PathBuildError"], match="ABSOLUTE"):
            opt.apply(dummy)

    @requires_libstp
    def test_pass_after_terminal_raises(self):
        imp = _imports()
        terminal = _DummyPass(terminal=True)
        terminal.name = "terminator"
        second = _DummyPass()
        second.name = "second"
        opt = imp["optimize"]([imp["drive_forward"](20)])
        opt.apply(terminal)
        with pytest.raises(imp["PathBuildError"], match="terminal"):
            opt.apply(second)

    @requires_libstp
    def test_either_requiring_pass_is_accepted(self):
        imp = _imports()
        dummy = _DummyPass(requires=imp["Representation"].EITHER)
        opt = imp["optimize"]([imp["drive_forward"](20)]).apply(dummy)
        assert opt._passes == [dummy]


# ---------------------------------------------------------------------------
# explain()
# ---------------------------------------------------------------------------


class TestExplain:
    @requires_libstp
    def test_explain_contains_each_pass_name_and_linear(self):
        imp = _imports()
        opt = imp["optimize"](
            [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](40)]
        ).cut_corners(5)
        text = opt.explain()
        # Always-on stages are rendered.
        assert "after decompose" in text
        assert "after merge" in text
        # The chained pass is rendered too.
        assert "corner_cut" in text
        assert "linear" in text

    @requires_libstp
    def test_explain_shows_always_on_without_user_passes(self):
        imp = _imports()
        opt = imp["optimize"]([imp["drive_forward"](20), imp["drive_forward"](30)])
        text = opt.explain()
        assert "after decompose" in text
        assert "after merge" in text


# ---------------------------------------------------------------------------
# Drop-in (no passes)
# ---------------------------------------------------------------------------


class TestDropIn:
    @requires_libstp
    def test_no_pass_optimizer_is_a_step(self):
        imp = _imports()
        opt = imp["optimize"](
            [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](30)]
        )
        assert isinstance(opt, imp["Step"])

    @requires_libstp
    def test_no_pass_compiled_nodes_equal_flatten(self):
        imp = _imports()
        steps = [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](30)]
        opt = imp["optimize"](list(steps))
        compiled = _compiled_nodes(opt)
        flat, _ = imp["flatten_steps"](list(steps))
        assert compiled == flat


# ---------------------------------------------------------------------------
# Representation guard + splinify message hygiene (the bug fix)
# ---------------------------------------------------------------------------


class _FakeIRSensor:
    """Minimal IRSensor stand-in for ``over_line``."""

    def probabilityOfBlack(self) -> float:
        return 0.0

    def probabilityOfWhite(self) -> float:
        return 1.0


class TestRepresentationGuard:
    @requires_libstp
    def test_to_absolute_then_splinify_raises_build_error(self):
        """to_absolute() produces ABSOLUTE; splinify() requires RELATIVE.

        The guard fires at BUILD time with a clean PathBuildError, not a raw
        ValueError from build_spline_step.
        """
        imp = _imports()
        opt = imp["optimize"]([imp["drive_forward"](50), imp["drive_forward"](30)]).to_absolute()
        with pytest.raises(imp["PathBuildError"]) as excinfo:
            opt.splinify()
        msg = str(excinfo.value)
        assert "splinify" in msg
        assert "RELATIVE" in msg
        assert "ABSOLUTE" in msg

    @requires_libstp
    def test_cut_corners_then_splinify_value_error_message(self):
        """cut_corners() stays RELATIVE, so splinify() reaches compile and the
        ValueError from build_spline_step must name splinify(), not smooth_path.
        """
        from raccoon.step.motion.path.compiler import PathCompiler

        imp = _imports()
        opt = (
            imp["optimize"](
                [imp["drive_forward"](50), imp["turn_right"](90), imp["drive_forward"](40)]
            )
            .cut_corners(5)
            .splinify()
        )
        with pytest.raises(ValueError) as excinfo:
            PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
        msg = str(excinfo.value)
        assert "splinify()" in msg
        assert "smooth_path" not in msg


# ---------------------------------------------------------------------------
# Always-on decompose (no .decompose() method)
# ---------------------------------------------------------------------------


class TestAlwaysOnDecompose:
    @requires_libstp
    def test_after_cm_plus_sensor_split_without_decompose(self):
        """An after_cm + sensor leg splits into TWO segments with no .decompose()."""
        from raccoon.step.condition import after_cm, over_line

        imp = _imports()
        sensor = _FakeIRSensor()
        cond = after_cm(12) + over_line(sensor)
        opt = imp["optimize"]([imp["drive_forward"]().until(cond)])
        nodes = _effective_compiled_nodes(opt)
        segs = [n for n in nodes if isinstance(n, imp["Segment"])]
        assert len(segs) == 2
        known, rest = segs
        assert known.has_known_endpoint is True
        assert abs(known.distance_m - 0.12) < 1e-9
        assert rest.has_known_endpoint is False
