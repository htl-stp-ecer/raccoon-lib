"""The ahead-of-time check for a missing heading reference."""

from __future__ import annotations

import importlib.util

import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("raccoon") is None,
    reason="raccoon native module not installed",
)


def _check(*roots):
    from raccoon.step.heading_check import HeadingReferenceChecker

    checker = HeadingReferenceChecker()
    for i, root in enumerate(roots):
        checker.check(root, f"mission {i}")


def _needs_reference():
    from raccoon.step import on_incline, on_level
    from raccoon.step.motion import (
        drive_forward,
        spline,
        turn_to_heading_right,
        wait_until_degrees,
    )
    from raccoon.step.motion.at_heading import HeadingOrigin

    return {
        "turn_to_heading": lambda: turn_to_heading_right(90),
        "drive_heading": lambda: drive_forward(10, heading=0),
        "spline_waypoints": lambda: spline((10, 0, 0), (20, 0, 0), absolute_heading=True),
        "spline_final": lambda: spline(
            (10, 0), (20, 0), absolute_heading=True, final_heading=0
        ),
        "wait_until_degrees": lambda: wait_until_degrees(
            45, origin=HeadingOrigin.HEADING_REFERENCE
        ),
        "tilt_condition": lambda: drive_forward(10).until(on_incline()),
        "combined_condition": lambda: drive_forward(10).until(on_level() | on_incline()),
    }


NAMES = [
    "turn_to_heading",
    "drive_heading",
    "spline_waypoints",
    "spline_final",
    "wait_until_degrees",
    "tilt_condition",
    "combined_condition",
]


@pytest.mark.parametrize("name", NAMES)
def test_use_without_mark_is_reported(name):
    from raccoon.step import seq

    with pytest.raises(RuntimeError, match="Call mark_heading_reference.*mission 0"):
        _check(seq([_needs_reference()[name]()]))


@pytest.mark.parametrize("name", NAMES)
def test_use_after_mark_passes(name):
    from raccoon.step import seq
    from raccoon.step.motion import mark_heading_reference

    _check(seq([mark_heading_reference(), _needs_reference()[name]()]))


def test_options_without_reference_need_none():
    from raccoon.step import seq
    from raccoon.step.motion import drive_forward, spline

    _check(
        seq(
            [
                drive_forward(10),
                spline((10, 0), (20, 0), absolute_heading=True),
                spline((10, 0, 0), (20, 0, 45)),
            ]
        )
    )


def test_mark_in_an_earlier_mission_counts():
    from raccoon.step import seq
    from raccoon.step.motion import mark_heading_reference, turn_to_heading_right

    _check(seq([mark_heading_reference()]), seq([turn_to_heading_right(90)]))


def test_mark_after_the_use_does_not_count():
    from raccoon.step import seq
    from raccoon.step.motion import mark_heading_reference, turn_to_heading_right

    with pytest.raises(RuntimeError, match="mission 0"):
        _check(seq([turn_to_heading_right(90)]), seq([mark_heading_reference()]))


def test_nested_composites_are_walked():
    from raccoon.step import loop_for, parallel, seq, timeout, wait_for_seconds
    from raccoon.step.motion import turn_to_heading_right

    tree = parallel(
        [seq([loop_for(timeout(turn_to_heading_right(90), 1.0), 2)]), wait_for_seconds(1)]
    )

    with pytest.raises(RuntimeError, match="TurnToHeading"):
        _check(tree)


def test_mark_in_a_concurrent_branch_is_not_reported():
    from raccoon.step import parallel, seq, wait_for_seconds
    from raccoon.step.motion import mark_heading_reference, turn_to_heading_right

    _check(
        parallel(
            [
                seq([wait_for_seconds(1), turn_to_heading_right(90)]),
                mark_heading_reference(),
            ]
        )
    )


@pytest.mark.parametrize("opaque", ["run", "defer", "custom"])
def test_nothing_is_reported_after_unknown_code(opaque):
    from raccoon.step import Step, defer, run, seq, wait_for_seconds
    from raccoon.step.motion import turn_to_heading_right

    class _Custom(Step):
        def _generate_signature(self):
            return "Custom()"

    steps = {
        "run": lambda: run(lambda robot: None),
        "defer": lambda: defer(lambda robot: wait_for_seconds(1)),
        "custom": _Custom,
    }

    _check(seq([steps[opaque](), turn_to_heading_right(90)]))


def test_closed_env_gate_is_skipped(monkeypatch):
    from raccoon.step import run_if_env, seq
    from raccoon.step.motion import turn_to_heading_right

    monkeypatch.delenv("HEADING_CHECK_TEST", raising=False)
    tree = seq([run_if_env(turn_to_heading_right(90), "HEADING_CHECK_TEST")])
    _check(tree)

    monkeypatch.setenv("HEADING_CHECK_TEST", "1")
    with pytest.raises(RuntimeError):
        _check(tree)


def test_preload_reports_the_missing_reference_before_any_mission_runs():
    from raccoon.mission.api import Mission
    from raccoon.robot.api import GenericRobot
    from raccoon.step import seq
    from raccoon.step.motion import turn_to_heading_right

    class _Mission(Mission):
        def sequence(self):
            return seq([turn_to_heading_right(90)])

    class _Robot:
        _pre_start_gate = GenericRobot._pre_start_gate

        def info(self, _msg):
            pass

        debug = warn = info

    with pytest.raises(RuntimeError, match="mission _Mission"):
        GenericRobot._preload_missions(_Robot(), [_Mission()], None, None)
