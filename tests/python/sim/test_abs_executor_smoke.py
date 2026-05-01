"""Sim smoke for :class:`AbsolutePathExecutor` — Phase-3 placeholder.

Phase 3 dispatches absolute Goto/TurnTo nodes by reading the world pose
and delegating to the relative motion DSL. Until Phase 4 wires the
motion classes themselves to consume absolute targets, end-pose
tolerances after a multi-leg plan are unstable: each motion still calls
``odometry.reset()`` in its ``start()``, and the heading PID baseline
shifts in ways the dumb dispatcher does not account for.

The skip is per the explicit Phase-3 out-clause in the migration plan
and the implementation task brief. Re-enable this test in Phase 4 when
``WorldCorrectionMiddleware`` is removed and motions are absolute-aware.
"""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.skip(
    reason=(
        "Phase 4 wires motion classes to absolute targets; until then end-pose "
        "tolerance after a multi-leg absolute plan is unstable. See "
        "docs/design/absolute-motion-plan.md (Phase 4)."
    ),
)


def test_absolute_executor_three_legs() -> None:
    """Reserved for Phase 4: goto(80,0) → turn_to(90) → goto(80,50)."""
    msg = "re-enable in Phase 4"
    raise AssertionError(msg)
