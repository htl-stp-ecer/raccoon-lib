"""Spline runs through the unified executor loop, not a special-case shortcut.

``smooth_path(spline=True)`` used to compile the path into a separate
``SplinePath`` step stored as ``CompiledPlan.spline_step`` and delegated to via
a dedicated shortcut in ``PathExecutor.run()``.  That shortcut has been removed:
the spline now lowers to a single ``Segment(kind="spline")`` node that the main
control loop drives like any other opaque segment.
"""

from __future__ import annotations

import importlib.util
import inspect

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


@requires_libstp
def test_spline_compiles_to_single_spline_segment_node():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.smooth_path import smooth_path
    from raccoon.step.motion.spline_path import SplinePath
    from raccoon.step.motion.turn_dsl import turn_right

    step = smooth_path(drive_forward(50), turn_right(90), drive_forward(30), spline=True)

    nodes = step._plan.nodes
    assert len(nodes) == 1
    seg = nodes[0]
    assert isinstance(seg, Segment)
    assert seg.kind == "spline"
    assert seg.has_known_endpoint is True
    # The SplinePath travels along as the opaque step the executor drives.
    assert isinstance(seg.opaque_step, SplinePath)

    # Spline mode lowers to a single opaque segment with no deferred nodes.
    assert step._plan.deferred == []


@requires_libstp
def test_spline_does_not_use_separate_spline_step_attribute():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.smooth_path import smooth_path
    from raccoon.step.motion.turn_dsl import turn_right

    step = smooth_path(drive_forward(50), turn_right(90), drive_forward(30), spline=True)

    # The collapsed-spline shortcut is gone — no separate step attribute.
    assert not hasattr(step, "_spline_step")


@requires_libstp
def test_compiled_plan_has_no_spline_step_field():
    from raccoon.step.motion.path.compiler import CompiledPlan

    assert "spline_step" not in inspect.signature(CompiledPlan.__init__).parameters
    assert not hasattr(CompiledPlan, "spline_step")


@requires_libstp
def test_executor_has_no_spline_step_param_or_attribute():
    from raccoon.step.motion.path.executor import PathExecutor

    params = inspect.signature(PathExecutor.__init__).parameters
    assert "spline_step" not in params

    executor = PathExecutor(nodes=[], deferred=[])
    assert not hasattr(executor, "_spline_step")
