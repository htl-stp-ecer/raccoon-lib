"""Construction-level tests for the absolute-pose plan IR (Phase 3, Commit A).

These tests cover only the IR dataclasses and the cm/deg → m/rad factory
layer. The compiler and executor land in later commits and are not
exercised here.
"""

from __future__ import annotations

import math

import pytest

from raccoon.step.motion.path import (
    Action,
    Goto,
    Resync,
    TurnTo,
    action,
    goto,
    resync,
    turn_to,
)

TOL = 1e-9


# --- factory unit conversions -------------------------------------------------


def test_goto_converts_cm_to_metres() -> None:
    g = goto(x_cm=80, y_cm=10)
    assert g.x_m == pytest.approx(0.80, abs=TOL)
    assert g.y_m == pytest.approx(0.10, abs=TOL)
    assert g.theta_rad is None
    assert g.via == "auto"
    assert g.speed_scale == 1.0


def test_goto_theta_deg_none_round_trips_to_none() -> None:
    g = goto(x_cm=0, y_cm=0)
    assert g.theta_rad is None


def test_goto_theta_deg_converts_to_radians() -> None:
    g = goto(x_cm=0, y_cm=0, theta_deg=180.0)
    assert g.theta_rad == pytest.approx(math.pi, abs=TOL)


def test_turn_to_converts_deg_to_radians() -> None:
    t = turn_to(theta_deg=90)
    assert isinstance(t, TurnTo)
    assert t.theta_rad == pytest.approx(math.pi / 2.0, abs=TOL)


def test_resync_converts_units_and_keeps_method() -> None:
    r = resync(
        "wall_align",
        expected_x_cm=30,
        expected_theta_deg=90,
        snap_axes=(True, False, True),
    )
    assert isinstance(r, Resync)
    assert r.method == "wall_align"
    assert r.expected_x_m == pytest.approx(0.30, abs=TOL)
    assert r.expected_y_m is None
    assert r.expected_theta_rad == pytest.approx(math.pi / 2.0, abs=TOL)
    assert r.snap_axes == (True, False, True)


def test_resync_defaults_are_optional() -> None:
    r = resync("io_button")
    assert r.expected_x_m is None
    assert r.expected_y_m is None
    assert r.expected_theta_rad is None
    assert r.snap_axes == (True, True, True)


# --- Action wraps an arbitrary Step-like object ------------------------------


class _MockStep:
    """Stand-in for a Step. The IR uses TYPE_CHECKING-only annotation."""


def test_action_wraps_step_blocking_default_true() -> None:
    s = _MockStep()
    a = action(s)
    assert isinstance(a, Action)
    assert a.step is s
    assert a.blocking is True


def test_action_blocking_false_passes_through() -> None:
    a = action(_MockStep(), blocking=False)
    assert a.blocking is False


# --- frozen / equality semantics ---------------------------------------------


def test_goto_is_frozen() -> None:
    g = Goto(x_m=1.0, y_m=0.0)
    with pytest.raises(Exception):  # FrozenInstanceError (subclass of Exception)
        g.x_m = 2.0  # type: ignore[misc]


def test_turn_to_is_frozen() -> None:
    t = TurnTo(theta_rad=0.0)
    with pytest.raises(Exception):
        t.theta_rad = 1.0  # type: ignore[misc]


def test_resync_is_frozen() -> None:
    r = Resync(method="marker")
    with pytest.raises(Exception):
        r.method = "find_line"  # type: ignore[misc]


def test_action_is_frozen() -> None:
    a = Action(step=_MockStep())
    with pytest.raises(Exception):
        a.blocking = False  # type: ignore[misc]


def test_goto_equality_by_value() -> None:
    a = Goto(x_m=0.5, y_m=0.25, theta_rad=math.pi)
    b = Goto(x_m=0.5, y_m=0.25, theta_rad=math.pi)
    assert a == b
    assert hash(a) == hash(b)


def test_turn_to_equality_by_value() -> None:
    assert TurnTo(theta_rad=1.0) == TurnTo(theta_rad=1.0)


def test_resync_equality_by_value() -> None:
    a = Resync(method="wall_align", expected_x_m=0.3)
    b = Resync(method="wall_align", expected_x_m=0.3)
    assert a == b


# --- import smoke (catches re-export drift) ----------------------------------


def test_public_names_importable_from_path_package() -> None:
    # Re-imports here to assert the names live on the package surface, not
    # only inside abs_ir / abs_factory.
    from raccoon.step.motion.path import (  # noqa: F401
        AbsoluteNode,
        Action,
        Goto,
        Resync,
        TurnTo,
        action,
        goto,
        resync,
        turn_to,
    )
