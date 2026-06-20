"""Unit tests for the centripetal Catmull-Rom math in
``raccoon.step.motion.path.passes.spline``.

These cover the pure-math core (``_catmull_rom_segment``,
``sample_centripetal_catmull_rom``, ``_arc_samples``,
``segments_to_spline_waypoints``) plus ``build_spline_step`` validation and the
``SplinifyPass`` render modes — no robot, no C++ HAL needed.

Assertions pin EXACT numeric values (tight float tolerances) and EXACT error
messages so mutations to coefficients, signs, operators, knot spacing, sample
counts and string constants all fail.
"""

from __future__ import annotations

import math

import pytest

from raccoon.motion import LinearAxis
from raccoon.step.motion.path.ir import Segment, SideAction
from raccoon.step.motion.path.passes.contract import Representation
from raccoon.step.motion.path.passes.spline import (
    SplinifyPass,
    _arc_samples,
    _catmull_rom_segment,
    build_spline_step,
    sample_centripetal_catmull_rom,
    segments_to_spline_waypoints,
)

ABS = 1e-9


# --------------------------------------------------------------------------- #
# _catmull_rom_segment — interpolation at the knots
# --------------------------------------------------------------------------- #


def test_segment_passes_through_p1_at_t0():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 1.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.0)
    assert x == pytest.approx(1.0, abs=ABS)
    assert y == pytest.approx(0.0, abs=ABS)


def test_segment_passes_through_p2_at_t1():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 1.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 1.0)
    assert x == pytest.approx(2.0, abs=ABS)
    assert y == pytest.approx(1.0, abs=ABS)


def test_segment_interior_point_is_between_endpoints():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 1.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    assert 1.0 < x < 2.0
    assert 0.0 < y < 1.0


def test_segment_collinear_is_a_straight_line():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    assert x == pytest.approx(1.5, abs=ABS)
    assert y == pytest.approx(0.0, abs=ABS)


def test_segment_collinear_quarter_point():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.25)
    assert x == pytest.approx(1.25, abs=ABS)
    assert y == pytest.approx(0.0, abs=ABS)


def test_segment_exact_values_asymmetric_polygon():
    # Pin the EXACT interior values for a non-collinear, non-symmetric polygon.
    # These bake in the centripetal (sqrt-chord) knot spacing AND the precise
    # Barry-Goldman lerp blend, so ANY change to the knot formula
    # (t_prev + sqrt(max(d, 1e-12))), the t0 base, or the lerp weights moves
    # these numbers.
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (1.0, 3.0), (4.0, 3.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    assert x == pytest.approx(0.9497595264191645, abs=1e-12)
    assert y == pytest.approx(1.4497595264191645, abs=1e-12)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.25)
    assert x == pytest.approx(1.08410446722156, abs=1e-12)
    assert y == pytest.approx(0.5528544672215598, abs=1e-12)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.75)
    assert x == pytest.approx(0.8405348224071867, abs=1e-12)
    assert y == pytest.approx(2.371784822407187, abs=1e-12)


def test_segment_knot_uses_sqrt_chord_not_chord():
    # Long P1->P2 chord with bent neighbours: pin the exact t=0.5 sample, which
    # depends on the sqrt-chord knot spacing (uniform/chord would differ).
    p0 = (0.0, 0.0)
    p1 = (0.0, 0.0)
    p2 = (9.0, 0.0)
    p3 = (9.0, 1.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    assert x == pytest.approx(4.218750374999875, abs=1e-9)
    assert y == pytest.approx(-0.28125000000000006, abs=1e-9)


# --------------------------------------------------------------------------- #
# coincident / d==0 divide-by-zero guard
# --------------------------------------------------------------------------- #


def test_segment_coincident_p1_p2_no_division_by_zero():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    assert math.isfinite(x) and math.isfinite(y)
    assert x == pytest.approx(1.0, abs=ABS)
    assert y == pytest.approx(0.0, abs=ABS)


def test_segment_coincident_endpoints_still_hit_at_knots():
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0)
    x0, y0 = _catmull_rom_segment(p0, p1, p2, p3, 0.0)
    x1, y1 = _catmull_rom_segment(p0, p1, p2, p3, 1.0)
    assert (x0, y0) == pytest.approx((1.0, 0.0), abs=ABS)
    assert (x1, y1) == pytest.approx((1.0, 0.0), abs=ABS)


def test_segment_all_coincident_no_crash():
    p = (2.0, -3.0)
    for t in (0.0, 0.5, 1.0):
        x, y = _catmull_rom_segment(p, p, p, p, t)
        assert math.isfinite(x) and math.isfinite(y)
        assert (x, y) == pytest.approx(p, abs=ABS)


def test_segment_coincident_neighbor_p0_eq_p1():
    p0, p1, p2, p3 = (0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (2.0, 1.0)
    x0, y0 = _catmull_rom_segment(p0, p1, p2, p3, 0.0)
    x1, y1 = _catmull_rom_segment(p0, p1, p2, p3, 1.0)
    assert math.isfinite(x0) and math.isfinite(x1)
    assert (x0, y0) == pytest.approx((0.0, 0.0), abs=ABS)
    assert (x1, y1) == pytest.approx((1.0, 1.0), abs=ABS)


def test_segment_guard_uses_floor_near_coincident():
    # A near-but-not-coincident chord (d slightly above the floor) still yields
    # a finite, well-defined interior point distinct from the endpoint.
    p0, p1, p2, p3 = (0.0, 0.0), (1.0, 0.0), (1.0, 1e-6), (2.0, 1e-6)
    x, y = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    assert math.isfinite(x) and math.isfinite(y)
    assert (x, y) != pytest.approx((1.0, 0.0), abs=1e-9)


# --------------------------------------------------------------------------- #
# centripetal (sqrt chord) knot spacing — proven against independent references
# --------------------------------------------------------------------------- #
#
# Independent Barry-Goldman Catmull-Rom evaluator, parameterised by the knot
# increment. Centripetal (alpha=0.5) uses knot_step = sqrt(chord); uniform
# (alpha=0) uses knot_step = 1. This is a from-scratch reimplementation (not a
# call into the source) so it can act as an oracle: the source must match the
# CENTRIPETAL branch and differ from the UNIFORM branch on unequal chords.


def _reference_catmull_rom_segment(p0, p1, p2, p3, t, *, centripetal):
    def knot(a, b, t_prev):
        if centripetal:
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            return t_prev + math.sqrt(max(d, 1e-12))
        return t_prev + 1.0  # uniform parameterisation (alpha = 0)

    t0 = 0.0
    t1 = knot(p0, p1, t0)
    t2 = knot(p1, p2, t1)
    t3 = knot(p2, p3, t2)
    u = t1 + t * (t2 - t1)

    def lerp(a, b, ta, tb):
        denom = tb - ta
        if abs(denom) < 1e-12:
            return a
        alpha = (u - ta) / denom
        return (
            (1.0 - alpha) * a[0] + alpha * b[0],
            (1.0 - alpha) * a[1] + alpha * b[1],
        )

    a1 = lerp(p0, p1, t0, t1)
    a2 = lerp(p1, p2, t1, t2)
    a3 = lerp(p2, p3, t2, t3)
    b1 = lerp(a1, a2, t0, t2)
    b2 = lerp(a2, a3, t1, t3)
    return lerp(b1, b2, t1, t2)


# Polygon with deliberately UNEQUAL chord lengths around the central segment:
# |P0P1| = 1, |P1P2| = 4, |P2P3| = sqrt(2). On equal chords centripetal and
# uniform coincide, so the inequality is what makes the two parameterisations
# diverge.
_UNEQUAL_CHORDS = ((0.0, 0.0), (1.0, 0.0), (5.0, 0.0), (6.0, 1.0))


def test_source_matches_centripetal_reference_not_uniform_on_unequal_chords():
    # The source's _catmull_rom_segment must reproduce the CENTRIPETAL oracle
    # exactly (to float precision) AND differ measurably from the UNIFORM oracle
    # at the same t — proving it really uses sqrt-chord (alpha=0.5) knot spacing
    # rather than uniform (alpha=0). On these unequal chords the two oracles are
    # themselves distinct, so the match-one/differ-from-the-other pair is a true
    # discriminator (not satisfied by both parameterisations).
    p0, p1, p2, p3 = _UNEQUAL_CHORDS
    for t in (0.25, 0.5, 0.75):
        got = _catmull_rom_segment(p0, p1, p2, p3, t)
        cent = _reference_catmull_rom_segment(p0, p1, p2, p3, t, centripetal=True)
        unif = _reference_catmull_rom_segment(p0, p1, p2, p3, t, centripetal=False)

        # Source == centripetal reference (tight tolerance).
        assert got[0] == pytest.approx(cent[0], abs=1e-12)
        assert got[1] == pytest.approx(cent[1], abs=1e-12)

        # Source != uniform reference (measurable separation), so it cannot be a
        # uniform Catmull-Rom in disguise.
        sep = math.hypot(got[0] - unif[0], got[1] - unif[1])
        assert sep > 1e-2

    # At the segment midpoint the centripetal-vs-uniform separation is ~0.071 m.
    mid_got = _catmull_rom_segment(p0, p1, p2, p3, 0.5)
    mid_unif = _reference_catmull_rom_segment(p0, p1, p2, p3, 0.5, centripetal=False)
    mid_sep = math.hypot(mid_got[0] - mid_unif[0], mid_got[1] - mid_unif[1])
    assert mid_sep == pytest.approx(0.07095062424038194, abs=1e-9)


def test_knot_step_is_sqrt_chord_via_t_image_on_unequal_chords():
    # Centripetal and uniform place the SAME local t at DIFFERENT world points
    # whenever the bracketing chords are unequal, because the interior knot
    # interval [t1, t2] maps t -> u = t1 + t*(t2 - t1) and the surrounding knot
    # spacing differs (sqrt-chord vs constant 1). Sampling several interior t's
    # and showing the source tracks the sqrt-chord image (centripetal) but NOT
    # the constant-step image (uniform) pins the knot formula to sqrt(chord).
    p0, p1, p2, p3 = _UNEQUAL_CHORDS
    max_match_cent = 0.0
    min_diff_unif = float("inf")
    for k in range(1, 10):
        t = k / 10.0
        got = _catmull_rom_segment(p0, p1, p2, p3, t)
        cent = _reference_catmull_rom_segment(p0, p1, p2, p3, t, centripetal=True)
        unif = _reference_catmull_rom_segment(p0, p1, p2, p3, t, centripetal=False)
        max_match_cent = max(max_match_cent, math.hypot(got[0] - cent[0], got[1] - cent[1]))
        min_diff_unif = min(min_diff_unif, math.hypot(got[0] - unif[0], got[1] - unif[1]))
    # Perfectly tracks centripetal across the whole interior...
    assert max_match_cent < 1e-12
    # ...and is never confusable with uniform at any interior sample.
    assert min_diff_unif > 1e-2


# --------------------------------------------------------------------------- #
# sample_centripetal_catmull_rom — endpoints, spacing, interpolation
# --------------------------------------------------------------------------- #


def test_sample_requires_at_least_two_points():
    with pytest.raises(ValueError, match=r"at least 2"):
        sample_centripetal_catmull_rom([(0.0, 0.0)])


def test_sample_empty_raises():
    with pytest.raises(ValueError, match=r"at least 2"):
        sample_centripetal_catmull_rom([])


def test_sample_starts_on_first_control_point():
    cps = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.3)
    assert pts[0] == pytest.approx((0.0, 0.0), abs=ABS)


def test_sample_ends_on_last_control_point():
    cps = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.3)
    assert pts[-1] == pytest.approx((2.0, 1.0), abs=ABS)


def test_sample_first_point_is_first_segment_start_not_shifted():
    # The seed sample is _catmull_rom_segment(points[0..3], 0.0), which is the
    # FIRST control point exactly. Mutants that swap points[0]->points[1] or
    # points[2]->points[3] in that seed would move the first sample off (0,0).
    cps = [(0.0, 0.0), (2.0, 1.0), (4.0, 0.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.3)
    assert pts[0] == pytest.approx((0.0, 0.0), abs=1e-12)
    assert pts[1] != pytest.approx((0.0, 0.0), abs=1e-6)


def test_sample_passes_through_interior_control_points_at_knots():
    cps = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (3.0, 1.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=1e9)
    assert len(pts) == len(cps)
    for got, want in zip(pts, cps):
        assert got == pytest.approx(want, abs=ABS)


def test_sample_first_segment_interior_exact_values():
    # Pin exact interior samples of the first segment for a 3-point polygon.
    # These depend on virtual_start = 2*P0 - P1 (reflection). Any change to
    # the 2.0 coefficients of virtual_start (x or y) shifts the first-segment
    # shape and breaks these numbers.
    cps = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    assert len(pts) == 11
    assert pts[1] == pytest.approx((0.43200000000000005, -0.032000000000000015), abs=1e-9)
    assert pts[2] == pytest.approx((0.896, -0.09599999999999996), abs=1e-9)
    assert pts[3] == pytest.approx((1.3439999999999999, -0.14399999999999993), abs=1e-9)
    assert pts[5] == pytest.approx((2.0, 0.0), abs=1e-9)


def test_sample_virtual_start_coefficients_nonzero_first_point():
    # virtual_start = (2*first[0] - second[0], 2*first[1] - second[1]). The
    # FIRST control point here has nonzero x AND y, so each coefficient (the
    # 2.0 on first[0] and the 2.0 on first[1]) and each index (first[0] vs
    # first[1]) is independently observable in the first-segment interior
    # samples. Pins virtual_start against 3*first[0], 2*first[1] (wrong index)
    # and 3*first[1] mutations.
    cps = [(1.0, 2.0), (3.0, 1.0), (5.0, 4.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    assert len(pts) == 14
    assert pts[0] == pytest.approx((1.0, 2.0), abs=1e-9)
    assert pts[1] == pytest.approx((1.405991343569011, 1.7525949638094482), abs=1e-9)
    assert pts[2] == pytest.approx((1.8179740307070331, 1.457784891428345), abs=1e-9)
    assert pts[3] == pytest.approx((2.2269610460605493, 1.1866773371425174), abs=1e-9)
    assert pts[4] == pytest.approx((2.623965374276044, 1.0103798552377936), abs=1e-9)
    # Last control point reached exactly.
    assert pts[-1] == pytest.approx((5.0, 4.0), abs=1e-9)


def test_sample_last_segment_interior_depends_on_virtual_end():
    # The tail samples depend on virtual_end = 2*P[-1] - P[-2]. Pin them so a
    # coefficient change there (the second leg's 2.0 factors) is caught.
    cps = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    assert pts[6] == pytest.approx((2.128, 0.27200000000000013), abs=1e-9)
    assert pts[7] == pytest.approx((2.144, 0.656), abs=1e-9)
    assert pts[8] == pytest.approx((2.096, 1.1039999999999999), abs=1e-9)
    assert pts[9] == pytest.approx((2.032, 1.5680000000000003), abs=1e-9)


def test_sample_two_collinear_points_is_straight_line():
    cps = [(0.0, 0.0), (1.0, 0.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    assert pts[0] == pytest.approx((0.0, 0.0), abs=ABS)
    assert pts[-1] == pytest.approx((1.0, 0.0), abs=ABS)
    for _x, y in pts:
        assert y == pytest.approx(0.0, abs=ABS)


def test_sample_spacing_count_two_points():
    cps = [(0.0, 0.0), (1.0, 0.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    assert len(pts) == 3
    assert pts[1] == pytest.approx((0.5, 0.0), abs=ABS)


def test_sample_coincident_points_no_crash():
    cps = [(1.0, 1.0), (1.0, 1.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    assert pts
    for x, y in pts:
        assert math.isfinite(x) and math.isfinite(y)
        assert (x, y) == pytest.approx((1.0, 1.0), abs=ABS)


def test_sample_consecutive_spacing_near_target():
    cps = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    spacing = 0.25
    pts = sample_centripetal_catmull_rom(cps, spacing_m=spacing)
    dists = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert dists
    avg = sum(dists) / len(dists)
    assert 0.5 * spacing <= avg <= 2.0 * spacing


def test_sample_default_spacing_used_when_omitted():
    cps = [(0.0, 0.0), (1.0, 0.0)]
    pts = sample_centripetal_catmull_rom(cps)
    assert len(pts) == 35
    assert pts[-1] == pytest.approx((1.0, 0.0), abs=ABS)


def test_sample_segment_count_uses_coarse_chord_length():
    # The per-segment sample count is ceil(coarse_chord_len / spacing). For a
    # straight 3 m polygon split into 3 segments, spacing 0.5 should give a
    # deterministic total count that pins the coarse-length accumulation
    # (sum of hypot over coarse=16 sub-steps) and the range(1, n+1) loops.
    cps = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    # 3 segments, each ~1 m -> ceil(1/0.5)=2 samples each = 6, + 1 seed = 7.
    assert len(pts) == 7
    xs = [round(p[0], 6) for p in pts]
    assert xs == [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    for _x, y in pts:
        assert y == pytest.approx(0.0, abs=ABS)


def test_sample_curve_bows_through_a_corner():
    cps = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.1)
    assert any(p == pytest.approx((1.0, 0.0), abs=1e-6) for p in pts)
    assert pts[0] == pytest.approx((0.0, 0.0), abs=ABS)
    assert pts[-1] == pytest.approx((1.0, 1.0), abs=ABS)


def test_sample_reflection_endpoint_tangent_direction():
    cps = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=0.5)
    for _x, y in pts:
        assert y == pytest.approx(0.0, abs=ABS)
    xs = [p[0] for p in pts]
    assert xs == sorted(xs)


def test_sample_negative_and_large_coordinates():
    cps = [(-1000.0, -1000.0), (0.0, 500.0), (1000.0, -1000.0)]
    pts = sample_centripetal_catmull_rom(cps, spacing_m=50.0)
    assert pts[0] == pytest.approx((-1000.0, -1000.0), abs=1e-6)
    assert pts[-1] == pytest.approx((1000.0, -1000.0), abs=1e-6)
    for x, y in pts:
        assert math.isfinite(x) and math.isfinite(y)


# --------------------------------------------------------------------------- #
# _arc_samples — ArcMotion geometry
# --------------------------------------------------------------------------- #


def test_arc_drive_ccw_quarter_turn_exact_samples():
    # Facing +x at origin, CCW (left) 90 deg drive arc, radius 1 m.
    # Centre (0,1); endpoint (1,1); heading -> +pi/2. num = ceil((pi/2)/18deg)=5.
    samples, xe, ye, he = _arc_samples(0.0, 0.0, 0.0, 1.0, math.pi / 2.0, False)
    assert xe == pytest.approx(1.0, abs=1e-9)
    assert ye == pytest.approx(1.0, abs=1e-9)
    assert he == pytest.approx(math.pi / 2.0, abs=1e-9)
    assert len(samples) == 5
    expected = [
        (0.3090169943749474, 0.04894348370484647),
        (0.5877852522924732, 0.19098300562505255),
        (0.8090169943749476, 0.41221474770752686),
        (0.9510565162951536, 0.6909830056250525),
        (1.0, 0.9999999999999999),
    ]
    for got, want in zip(samples, expected):
        assert got == pytest.approx(want, abs=1e-9)
    for px, py in samples:
        assert math.hypot(px - 0.0, py - 1.0) == pytest.approx(1.0, abs=1e-9)


def test_arc_drive_cw_quarter_turn():
    samples, xe, ye, he = _arc_samples(0.0, 0.0, 0.0, 1.0, -math.pi / 2.0, False)
    assert xe == pytest.approx(1.0, abs=1e-9)
    assert ye == pytest.approx(-1.0, abs=1e-9)
    assert he == pytest.approx(-math.pi / 2.0, abs=1e-9)
    for px, py in samples:
        assert math.hypot(px - 0.0, py + 1.0) == pytest.approx(1.0, abs=1e-9)


def test_arc_strafe_lateral_ccw_exact():
    # lateral=True, CCW: velocity_dir = heading + 90deg = +y. Centre another
    # 90deg left -> at (-1, 0). Endpoint after pi/2 sweep is (-1, 1).
    samples, xe, ye, he = _arc_samples(0.0, 0.0, 0.0, 1.0, math.pi / 2.0, True)
    assert he == pytest.approx(math.pi / 2.0, abs=1e-9)
    assert xe == pytest.approx(-1.0, abs=1e-9)
    assert ye == pytest.approx(1.0, abs=1e-9)
    assert samples[0] == pytest.approx((-0.04894348370484647, 0.3090169943749474), abs=1e-9)
    for px, py in samples:
        assert math.hypot(px + 1.0, py - 0.0) == pytest.approx(1.0, abs=1e-9)


def test_arc_strafe_lateral_cw_exact():
    # CW strafe: velocity_dir = heading - 90 = -y; centre another 90 right ->
    # at (-1, 0). Endpoint after -pi/2 sweep is (-1, -1).
    samples, xe, ye, he = _arc_samples(0.0, 0.0, 0.0, 1.0, -math.pi / 2.0, True)
    assert he == pytest.approx(-math.pi / 2.0, abs=1e-9)
    assert (xe, ye) == pytest.approx((-1.0, -1.0), abs=1e-9)
    assert samples[0] == pytest.approx((-0.04894348370484647, -0.3090169943749474), abs=1e-9)
    for px, py in samples:
        assert math.hypot(px + 1.0, py) == pytest.approx(1.0, abs=1e-9)


def test_arc_lateral_branch_distinguished_from_drive():
    # Same sweep/radius, lateral True vs False must produce DIFFERENT endpoints
    # (drive endpoint (1,1) vs strafe endpoint (-1,1)). Pins the
    # `heading if not lateral else ...` branch.
    drive = _arc_samples(0.0, 0.0, 0.0, 1.0, math.pi / 2.0, False)
    strafe = _arc_samples(0.0, 0.0, 0.0, 1.0, math.pi / 2.0, True)
    assert (drive[1], drive[2]) == pytest.approx((1.0, 1.0), abs=1e-9)
    assert (strafe[1], strafe[2]) == pytest.approx((-1.0, 1.0), abs=1e-9)


def test_arc_minimum_two_samples_for_tiny_sweep():
    samples, _xe, _ye, _he = _arc_samples(0.0, 0.0, 0.0, 1.0, 0.01, False)
    assert len(samples) == 2


def test_arc_sample_count_step_is_18_degrees():
    # num = max(2, ceil(sweep / 18deg)). A 38 deg sweep -> ceil(38/18)=3.
    # If the step constant were 19 deg, ceil(38/19)=2, so this pins 18.0 deg.
    samples, _xe, _ye, _he = _arc_samples(0.0, 0.0, 0.0, 1.0, math.radians(38.0), False)
    assert len(samples) == 3
    assert len(_arc_samples(0, 0, 0, 1.0, math.radians(36.0), False)[0]) == 2
    assert len(_arc_samples(0, 0, 0, 1.0, math.radians(57.0), False)[0]) == 4


def test_arc_zero_radius_stays_in_place():
    samples, xe, ye, he = _arc_samples(0.0, 0.0, 0.0, 0.0, math.pi / 2.0, False)
    assert (xe, ye) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert he == pytest.approx(math.pi / 2.0, abs=1e-9)
    for p in samples:
        assert p == pytest.approx((0.0, 0.0), abs=1e-9)


def test_arc_sign_positive_below_one_radian():
    # sign = 1.0 if arc_angle_rad >= 0.0 else -1.0. For a SMALL positive sweep
    # (0.5 rad < 1.0) the sign must be +1 (CCW): centre 90 deg LEFT at (0, 1),
    # so the robot curves toward +y. Pin the exact endpoint. A `>= 1.0` boundary
    # mutant would treat 0.5 rad as sign=-1 (centre at (0,-1) -> curve toward
    # -y), giving a different endpoint.
    samples, xe, ye, he = _arc_samples(0.0, 0.0, 0.0, 1.0, 0.5, False)
    assert he == pytest.approx(0.5, abs=1e-9)
    assert xe == pytest.approx(0.479425538604203, abs=1e-9)
    assert ye == pytest.approx(0.12241743810962724, abs=1e-9)
    # Curving CCW -> y strictly positive.
    assert ye > 0.0


def test_arc_zero_angle_endpoint_is_start():
    # At exactly arc_angle 0.0 the sweep is zero, so the robot does not move
    # regardless of the centre side; endpoint equals the start.
    samples, xe, ye, he = _arc_samples(1.0, 2.0, 0.3, 1.0, 0.0, False)
    assert (xe, ye) == pytest.approx((1.0, 2.0), abs=1e-9)
    assert he == pytest.approx(0.3, abs=1e-9)
    for p in samples:
        assert p == pytest.approx((1.0, 2.0), abs=1e-9)


# --------------------------------------------------------------------------- #
# segments_to_spline_waypoints — body-frame pose integration (returns cm)
# --------------------------------------------------------------------------- #


def test_waypoints_linear_forward_is_cm():
    segs = [Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0)]
    wps = segments_to_spline_waypoints(segs)
    assert len(wps) == 1
    assert wps[0] == pytest.approx((100.0, 0.0), abs=1e-6)


def test_waypoints_linear_lateral():
    segs = [Segment(kind="linear", axis=LinearAxis.Lateral, distance_m=0.5)]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 50.0), abs=1e-6)


def test_waypoints_forward_at_heading_accumulates_both_axes():
    # Turn +90 deg, then forward 1 m -> +y; then forward again must ACCUMULATE
    # (x += / y += not x = / y =). Two forward moves at heading +90 land at
    # (0, 200) cm. A `y =` mutant (line 203) would clobber to (0, 100).
    segs = [
        Segment(kind="turn", angle_rad=math.pi / 2.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 100.0), abs=1e-6)
    assert wps[1] == pytest.approx((0.0, 200.0), abs=1e-6)


def test_waypoints_forward_at_45deg_uses_cos_and_sin():
    segs = [
        Segment(kind="turn", angle_rad=math.pi / 4.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=math.sqrt(2.0)),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((100.0, 100.0), abs=1e-6)


def test_waypoints_lateral_at_heading_accumulates():
    # Forward 1 m, then lateral 1 m at heading 0: lateral +left -> +y.
    # Two segments so the lateral x += / y += accumulation (lines 206, 210) is
    # exercised: a `x =`/`y =` mutant changes (100,100).
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
        Segment(kind="linear", axis=LinearAxis.Lateral, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((100.0, 0.0), abs=1e-6)
    assert wps[1] == pytest.approx((100.0, 100.0), abs=1e-6)


def test_waypoints_lateral_sign_is_negative_sin_at_heading_90():
    # At heading +90 deg, lateral (+left) points to -x. 1 m lateral -> (-100, 0).
    # Pins the `-math.sin(heading)` (not +sin) for the lateral x update.
    segs = [
        Segment(kind="turn", angle_rad=math.pi / 2.0),
        Segment(kind="linear", axis=LinearAxis.Lateral, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((-100.0, 0.0), abs=1e-6)


def test_waypoints_lateral_y_accumulates_not_clobbered():
    # The lateral y update must be `y += d*cos(heading)`, NOT `y = ...`. Set y
    # nonzero first via a diagonal (heading 0): (1,1) m. Then a lateral 1 m at
    # heading 0 adds d*cos(0)=1 -> y must reach 200 cm. A `y = d*cos` clobber
    # (mutant 210) would reset y to 100 cm.
    segs = [
        Segment(kind="diagonal", forward_m=1.0, left_m=1.0),
        Segment(kind="linear", axis=LinearAxis.Lateral, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((100.0, 100.0), abs=1e-6)
    assert wps[1] == pytest.approx((100.0, 200.0), abs=1e-6)


def test_waypoints_diagonal_body_frame_exact():
    segs = [Segment(kind="diagonal", forward_m=1.0, left_m=0.5)]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((100.0, 50.0), abs=1e-6)


def test_waypoints_diagonal_at_heading_90():
    # diagonal forward 1, left 0.5 at heading +90 deg:
    # x += fwd*cos(90) - left*sin(90) = -0.5; y += fwd*sin(90) + left*cos(90) = 1.0
    # -> (-50, 100) cm. Pins both diagonal terms and their +/- signs.
    segs = [
        Segment(kind="turn", angle_rad=math.pi / 2.0),
        Segment(kind="diagonal", forward_m=1.0, left_m=0.5),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((-50.0, 100.0), abs=1e-6)


def test_waypoints_diagonal_accumulates():
    segs = [
        Segment(kind="diagonal", forward_m=1.0, left_m=0.5),
        Segment(kind="diagonal", forward_m=1.0, left_m=0.5),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((100.0, 50.0), abs=1e-6)
    assert wps[1] == pytest.approx((200.0, 100.0), abs=1e-6)


def test_waypoints_diagonal_none_forward_and_left_are_zero():
    segs = [Segment(kind="diagonal", forward_m=None, left_m=None)]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 0.0), abs=1e-9)


def test_waypoints_turn_then_forward():
    segs = [
        Segment(kind="turn", angle_rad=math.pi / 2.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert len(wps) == 1
    assert wps[0][0] == pytest.approx(0.0, abs=1e-6)
    assert wps[0][1] == pytest.approx(100.0, abs=1e-6)


def test_waypoints_turn_accumulates_heading():
    # Two +45 deg turns must SUM to +90 deg (heading += not heading =). After
    # them a forward 1 m lands at (0, 100). A `heading =` mutant (line 252)
    # would leave heading at the last turn's 45 deg -> (~70.7, ~70.7).
    segs = [
        Segment(kind="turn", angle_rad=math.pi / 4.0),
        Segment(kind="turn", angle_rad=math.pi / 4.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 100.0), abs=1e-6)


def test_waypoints_turn_none_angle_is_zero_heading_delta():
    # turn angle_rad None -> += 0.0. Forward stays along +x. A
    # `heading += angle or 1.0` mutant (line 254) would rotate by 1 rad.
    segs = [
        Segment(kind="turn", angle_rad=None),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((100.0, 0.0), abs=1e-6)


def test_waypoints_arc_emits_multiple_samples_exact_endpoint():
    segs = [Segment(kind="arc", radius_m=1.0, arc_angle_rad=math.pi / 2.0, lateral=False)]
    wps = segments_to_spline_waypoints(segs)
    assert len(wps) == 5
    assert wps[-1] == pytest.approx((100.0, 100.0), abs=1e-4)
    assert wps[0] == pytest.approx((30.90169943749474, 4.894348370484647), abs=1e-6)


def test_waypoints_arc_none_radius_is_zero():
    # radius_m None -> 0.0: arc stays in place, heading sweeps. A
    # `radius_m or 1.0` mutant would trace a unit-radius arc.
    segs = [
        Segment(kind="arc", radius_m=None, arc_angle_rad=math.pi / 2.0, lateral=False),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 0.0), abs=1e-9)
    assert wps[-1] == pytest.approx((0.0, 100.0), abs=1e-6)


def test_waypoints_arc_none_angle_is_zero_sweep():
    # arc_angle_rad None -> 0.0: min 2 samples, both at the start, heading
    # unchanged. A `arc_angle_rad or 1.0` mutant would sweep 1 rad.
    segs = [
        Segment(kind="arc", radius_m=1.0, arc_angle_rad=None, lateral=False),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 0.0), abs=1e-9)
    assert wps[1] == pytest.approx((0.0, 0.0), abs=1e-9)
    assert wps[-1] == pytest.approx((100.0, 0.0), abs=1e-6)


def test_waypoints_turn_is_last_segment_loops_back():
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
        Segment(kind="turn", angle_rad=math.pi / 4.0),
    ]
    wps = segments_to_spline_waypoints(segs)
    assert len(wps) == 1
    assert wps[0] == pytest.approx((100.0, 0.0), abs=1e-6)


def test_waypoints_none_distance_treated_as_zero():
    segs = [Segment(kind="linear", axis=LinearAxis.Forward, distance_m=None)]
    wps = segments_to_spline_waypoints(segs)
    assert wps[0] == pytest.approx((0.0, 0.0), abs=1e-9)


# --------------------------------------------------------------------------- #
# build_spline_step — validation + waypoint reuse
# --------------------------------------------------------------------------- #


def _two_forward_segs():
    return [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]


def test_build_spline_step_valid():
    sp = build_spline_step(_two_forward_segs())
    assert sp._waypoints == [(100.0, 0.0), (200.0, 0.0)]
    assert sp._speed == 1.0


def test_build_spline_step_speed_is_min_of_geometry_segments():
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0, speed_scale=0.8),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0, speed_scale=0.4),
    ]
    sp = build_spline_step(segs)
    assert sp._speed == pytest.approx(0.4)


def test_build_speed_min_includes_diagonal_segments():
    # geom_kinds must include "diagonal": a slow diagonal must drag the min
    # speed down. If "diagonal" were dropped (mutant 296), speed would be 0.9.
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0, speed_scale=0.9),
        Segment(kind="diagonal", forward_m=1.0, left_m=0.0, speed_scale=0.2),
    ]
    sp = build_spline_step(segs)
    assert sp._speed == pytest.approx(0.2)


def test_build_speed_min_includes_arc_segments():
    # geom_kinds must include "arc": a slow arc must drag the min speed down.
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0, speed_scale=0.9),
        Segment(
            kind="arc",
            radius_m=1.0,
            arc_angle_rad=math.pi / 2.0,
            lateral=False,
            speed_scale=0.3,
        ),
    ]
    sp = build_spline_step(segs)
    assert sp._speed == pytest.approx(0.3)


def test_build_speed_default_is_one():
    # The min(...) default kwarg is 1.0 (not 2.0). A path of geom segments all
    # at speed 1.0 stays exactly 1.0.
    sp = build_spline_step(_two_forward_segs())
    assert sp._speed == 1.0


def test_build_rejects_defer_none():
    with pytest.raises(ValueError) as exc:
        build_spline_step([None, *_two_forward_segs()])
    assert str(exc.value) == (
        "splinify() cannot contain Defer steps — "
        "waypoints must be fully known at construction time"
    )


def test_build_rejects_side_action():
    sa = SideAction(step=object(), is_background=False)
    with pytest.raises(ValueError) as exc:
        build_spline_step([sa, *_two_forward_segs()])
    assert str(exc.value) == (
        "splinify() cannot contain side actions " "(background(), Run, or non-drive steps)"
    )


def test_build_rejects_condition_segment():
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        distance_m=1.0,
        condition=object(),
    )
    with pytest.raises(ValueError) as exc:
        build_spline_step([seg])
    assert str(exc.value) == (
        "splinify() cannot contain condition-based " "segments (.until()) — endpoint must be known"
    )


def test_build_rejects_unknown_endpoint():
    seg = Segment(kind="linear", has_known_endpoint=False)
    with pytest.raises(ValueError) as exc:
        build_spline_step([seg])
    assert str(exc.value) == ("splinify() requires all segments to have known endpoints")


def test_build_rejects_follow_line():
    with pytest.raises(ValueError) as exc:
        build_spline_step([Segment(kind="follow_line")])
    assert str(exc.value) == (
        "splinify() cannot contain follow_line segments — "
        "endpoint must be a simple linear/turn sequence"
    )


def test_build_rejects_nested_spline():
    with pytest.raises(ValueError) as exc:
        build_spline_step([Segment(kind="spline")])
    assert str(exc.value) == (
        "splinify() cannot contain spline segments — "
        "endpoint must be a simple linear/turn sequence"
    )


def test_build_rejects_turn_to_heading_none_angle():
    with pytest.raises(ValueError) as exc:
        build_spline_step([Segment(kind="turn", angle_rad=None)])
    assert str(exc.value) == (
        "splinify() cannot contain turn_to_heading_* turns — they "
        "target an absolute reference heading, not a relative angle, "
        "so they cannot be folded into a spline"
    )


def test_build_rejects_turn_with_opaque_step():
    seg = Segment(kind="turn", angle_rad=math.pi / 2.0, opaque_step=object())
    with pytest.raises(ValueError) as exc:
        build_spline_step([seg])
    assert str(exc.value) == (
        "splinify() cannot contain turn_to_heading_* turns — they "
        "target an absolute reference heading, not a relative angle, "
        "so they cannot be folded into a spline"
    )


def test_build_rejects_too_few_waypoints():
    with pytest.raises(ValueError) as exc:
        build_spline_step([Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0)])
    assert str(exc.value) == (
        "splinify() requires at least 2 control waypoints " "to form a valid spline (found 1)"
    )


def test_build_accepts_relative_turn_between_linears():
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
        Segment(kind="turn", angle_rad=math.pi / 2.0),
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
    ]
    sp = build_spline_step(segs)
    assert len(sp._waypoints) == 2


# --------------------------------------------------------------------------- #
# SplinifyPass — relative vs absolute render modes
# --------------------------------------------------------------------------- #


def test_pass_metadata():
    p = SplinifyPass()
    assert p.name == "splinify"
    assert p.terminal is True
    assert p.absolute is False
    # requires must be the RELATIVE representation, not None.
    assert p.requires == Representation.RELATIVE
    assert p.requires is not None


def test_pass_relative_lowers_to_segments():
    p = SplinifyPass(absolute=False)
    out = p.run(_two_forward_segs())
    assert isinstance(out, list)
    assert all(isinstance(n, Segment) for n in out)
    assert any(getattr(n, "kind", None) == "spline" for n in out)


def test_pass_absolute_emits_inline_side_action_meters():
    p = SplinifyPass(absolute=True)
    out = p.run(_two_forward_segs())
    assert len(out) == 1
    node = out[0]
    assert isinstance(node, SideAction)
    assert node.is_background is False
    step = node.step
    assert step.__class__.__name__ == "SplineFollow"
    # Control points must be converted cm -> METRES (fwd/100, left/100).
    # Two 1 m forward moves -> (100,0),(200,0) cm -> (1,0),(2,0) m.
    wp = step._waypoints
    assert wp[0] == pytest.approx((1.0, 0.0), abs=1e-9)
    assert wp[1] == pytest.approx((2.0, 0.0), abs=1e-9)
    assert step._speed == pytest.approx(1.0)
    assert step._heading_mode == "hold"


def test_pass_absolute_lateral_conversion_is_per_axis():
    # A non-zero LEFT component proves left_cm/100 (not left_cm*100).
    # Forward 1 m then lateral 1 m -> (1,0),(1,1) m.
    p = SplinifyPass(absolute=True)
    segs = [
        Segment(kind="linear", axis=LinearAxis.Forward, distance_m=1.0),
        Segment(kind="linear", axis=LinearAxis.Lateral, distance_m=1.0),
    ]
    out = p.run(segs)
    wp = out[0].step._waypoints
    assert wp[0] == pytest.approx((1.0, 0.0), abs=1e-9)
    assert wp[1] == pytest.approx((1.0, 1.0), abs=1e-9)


def test_pass_passes_deferred_through():
    # A deferred (None) placeholder is a barrier: the pass leaves it in place
    # rather than raising — it can't be folded into a curve but no longer aborts
    # the whole splinify. build_spline_step (the strict single-spline builder
    # used by smooth_path) still raises on Defer — covered separately above.
    p = SplinifyPass(absolute=True)
    assert p.run([None]) == [None]
