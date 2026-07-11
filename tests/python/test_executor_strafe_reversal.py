"""Executor fixes for the strafe direction-reversal stall.

Reproduces (as pure-Python unit tests, no C++/sim) the two defects behind the
hardware log ``libstp-2026-07-01_11-12-58``, where a ``Lateral -5cm`` strafe
followed by a ``Lateral +6cm`` strafe hung until the stall watchdog skipped it:

1. **Warm-start across a reversal.** The +6cm strafe warm-started from the -5cm
   strafe (same type) and carried its LEFTWARD velocity, flying ~5.5cm the wrong
   way before reversing. Fixed by :func:`_should_warm` /
   :func:`_reverses_direction`: opposite signs on the same axis ⇒ cold-start.

2. **Profile ramps to zero, then stuck.** As a NON-last, NON-inflated segment
   the +6cm strafe decelerated to its target and the C++ motion self-completed
   at its 1cm tolerance, then hard-stopped. The executor's manual completion
   check demanded a tighter 5mm band, so it waited — profile parked at zero —
   until the 4s watchdog. Fixed by also accepting the motion's own completion
   (``_has_reached_target``) for non-last segments.
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


@pytest.fixture
def _mod():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path import executor as ex
    from raccoon.step.motion.path.ir import Segment

    return ex, Segment, LinearAxis


def _lin(mod_axis, axis, distance_m):
    _ex, Segment, LinearAxis = mod_axis
    return Segment(
        kind="linear",
        axis=axis,
        sign=1.0 if distance_m >= 0 else -1.0,
        distance_m=distance_m,
    )


# ---------------------------------------------------------------------------
# Fix 1 — no warm-start across a same-axis direction reversal
# ---------------------------------------------------------------------------


@requires_libstp
def test_reverses_direction_opposite_signs_same_axis(_mod):
    ex, _Segment, LinearAxis = _mod
    prev = _lin(_mod, LinearAxis.Lateral, -0.05)  # strafe left 5cm
    nxt = _lin(_mod, LinearAxis.Lateral, +0.06)  # strafe right 6cm
    assert ex._reverses_direction(prev, nxt) is True


@requires_libstp
def test_no_reversal_same_direction(_mod):
    ex, _Segment, LinearAxis = _mod
    prev = _lin(_mod, LinearAxis.Lateral, +0.05)
    nxt = _lin(_mod, LinearAxis.Lateral, +0.06)
    assert ex._reverses_direction(prev, nxt) is False


@requires_libstp
def test_no_reversal_when_distance_unknown(_mod):
    ex, _Segment, LinearAxis = _mod
    prev = _lin(_mod, LinearAxis.Lateral, -0.05)
    nxt = _lin(_mod, LinearAxis.Lateral, +0.06)
    nxt.distance_m = None  # condition-only (sentinel) leg → no sign to compare
    assert ex._reverses_direction(prev, nxt) is False


@requires_libstp
def test_should_warm_false_across_reversal(_mod):
    ex, _Segment, LinearAxis = _mod
    prev = _lin(_mod, LinearAxis.Lateral, -0.05)
    nxt = _lin(_mod, LinearAxis.Lateral, +0.06)
    # Same type (both Lateral) so is_same_type would say "warm", but the
    # reversal must force a cold start.
    assert ex._should_warm(prev, nxt) is False


@requires_libstp
def test_should_warm_true_same_direction(_mod):
    ex, _Segment, LinearAxis = _mod
    prev = _lin(_mod, LinearAxis.Lateral, +0.05)
    nxt = _lin(_mod, LinearAxis.Lateral, +0.06)
    assert ex._should_warm(prev, nxt) is True


@requires_libstp
def test_next_segment_warmstarts_false_before_reversal(_mod):
    ex, _Segment, LinearAxis = _mod
    prev = _lin(_mod, LinearAxis.Lateral, -0.05)
    nxt = _lin(_mod, LinearAxis.Lateral, +0.06)
    # The -5cm leg must NOT inflate/cruise, because the +6cm leg will cold-start
    # at the reversal — otherwise it overshoots its own endpoint.
    nodes = [prev, nxt]
    assert ex._next_segment_warmstarts(nodes, 0, prev) is False


# ---------------------------------------------------------------------------
# Fix 2 — non-last completion accepts the motion's OWN completion
# ---------------------------------------------------------------------------


class _FakeOdometry:
    def __init__(self, x, y, heading):
        from types import SimpleNamespace

        self._pose = SimpleNamespace(position=(x, y), heading=heading)
        self._heading = heading

    def get_pose(self):
        return self._pose

    def get_heading(self):
        return self._heading


class _FakeRobot:
    def __init__(self, x, y, heading=0.0):
        self.odometry = _FakeOdometry(x, y, heading)


class _FakeMotion:
    """Stands in for the C++ LinearMotion completion queries."""

    def __init__(self, reached):
        self._reached = reached

    def has_reached_distance(self):
        return self._reached


@requires_libstp
def test_non_last_transitions_when_motion_completes_short(_mod):
    """The exact hardware trap: robot parked at 5.22cm, target 6cm.

    The manual 2mm band (needs >=5.8cm) is NOT satisfied, but the motion
    reports completion. The executor MUST accept that completion and transition
    — never sit stuck at a ramped-to-zero profile until the watchdog.
    """
    ex, _Segment, LinearAxis = _mod
    seg = _lin(_mod, LinearAxis.Lateral, +0.06)
    # Lateral position projected on heading 0: lateral = -x*sin+y*cos = y.
    robot = _FakeRobot(x=0.0, y=0.0522)  # 5.22cm — short of the 5.8cm band
    seg_origin = 0.0

    # Manual check alone would keep waiting (the pre-fix bug):
    assert ex._check_segment_reached(robot, seg, seg_origin) is False
    # With the motion reporting completion, the executor transitions:
    motion = _FakeMotion(reached=True)
    assert ex._non_last_reached(robot, seg, seg_origin, motion) is True


@requires_libstp
def test_non_last_waits_while_inflated_profile_cruises(_mod):
    """Inflated segments are unaffected: the motion never self-completes.

    An inflated (warm-continuation) leg cruises through its endpoint; its
    ``has_reached_distance()`` checks the +1m inflated target and stays False.
    So the OR term is inert and only the manual position check drives the
    transition — preserving the existing warm-start cruise behaviour.
    """
    ex, _Segment, LinearAxis = _mod
    seg = _lin(_mod, LinearAxis.Lateral, +0.06)
    robot = _FakeRobot(x=0.0, y=0.03)  # 3cm — not yet at target
    seg_origin = 0.0

    motion = _FakeMotion(reached=False)  # inflated target never reached
    assert ex._non_last_reached(robot, seg, seg_origin, motion) is False


@requires_libstp
def test_non_last_transitions_on_manual_band_when_not_completed(_mod):
    """Manual band still works on its own (unchanged path)."""
    ex, _Segment, LinearAxis = _mod
    seg = _lin(_mod, LinearAxis.Lateral, +0.06)
    robot = _FakeRobot(x=0.0, y=0.059)  # 5.9cm — inside the 2mm band (>=5.8cm)
    seg_origin = 0.0

    motion = _FakeMotion(reached=False)
    assert ex._non_last_reached(robot, seg, seg_origin, motion) is True


# ---------------------------------------------------------------------------
# Fix 3 — the reversal hard stop is PHYSICALLY realised (settle to rest)
# ---------------------------------------------------------------------------
#
# hard_stop() only *commands* zero body velocity and returns while the robot is
# still coasting. Starting the opposite-direction leg immediately re-arms the
# motors mid-coast, so the stop never physically happens. Before a cold start we
# hold zero and pump the drive controllers until the carried velocity is bled
# off, so the new profile genuinely warm-starts from rest.


class _FakeDrive:
    """Counts the stop/update pumps a settle issues."""

    def __init__(self):
        self.hard_stops = 0
        self.updates = 0

    def hard_stop(self):
        self.hard_stops += 1

    def update(self, dt):
        self.updates += 1


class _FakeDriveRobot:
    def __init__(self):
        self.drive = _FakeDrive()


@requires_libstp
def test_settle_time_zero_when_at_rest(_mod):
    ex, _Segment, _LinearAxis = _mod
    assert ex._settle_time_s(0.0) == 0.0
    assert ex._settle_time_s(0.0005) == 0.0  # below the at-rest epsilon


@requires_libstp
def test_settle_time_scales_and_clamps(_mod):
    ex, _Segment, _LinearAxis = _mod
    # Full-speed reversal saturates at the cap.
    assert ex._settle_time_s(10.0) == ex._MAX_SETTLE_S
    # A tiny non-zero carry still gets the floor (the brake must register).
    assert ex._settle_time_s(0.01) == ex._MIN_SETTLE_S
    # In between: monotonic in speed, inside the bounds.
    mid = ex._settle_time_s(0.3)
    assert ex._MIN_SETTLE_S <= mid <= ex._MAX_SETTLE_S
    assert ex._settle_time_s(0.5) >= mid
    # Sign-independent — a leftward coast settles the same as rightward.
    assert ex._settle_time_s(-0.4) == ex._settle_time_s(0.4)


@requires_libstp
def test_settle_to_rest_noop_when_already_stopped(_mod):
    import asyncio

    ex, _Segment, _LinearAxis = _mod
    robot = _FakeDriveRobot()
    asyncio.run(ex._settle_to_rest(robot, hz=200.0, entry_vel=0.0))
    # Nothing to bleed off ⇒ no braking pumps, no added latency.
    assert robot.drive.hard_stops == 0
    assert robot.drive.updates == 0


@requires_libstp
def test_settle_to_rest_pumps_brake_when_coasting(_mod):
    import asyncio

    ex, _Segment, _LinearAxis = _mod
    robot = _FakeDriveRobot()
    asyncio.run(ex._settle_to_rest(robot, hz=500.0, entry_vel=0.4))
    # It actively held the stop and pumped the controllers at least once.
    assert robot.drive.hard_stops >= 1
    # hard_stop and update are pumped together on each active tick.
    assert robot.drive.updates == robot.drive.hard_stops
