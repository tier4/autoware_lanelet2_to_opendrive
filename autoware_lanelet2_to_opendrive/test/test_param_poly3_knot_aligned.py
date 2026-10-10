"""Knot-aligned, honestly-parameterised paramPoly3 emission.

Two defects are covered here.

**Defect 1 -- ``pRange`` told the truth about nothing.**  Every emitted
segment declared ``pRange="arcLength"``, which asserts that the curve
parameter *is* travelled distance, while the coefficients came from a cubic
Hermite fit that is not unit-speed.  ASAM's
``road.geometry.parampoly3.arclength_range`` checker integrates
``|(u', v')|`` over ``[0, @length]`` and flagged the mismatch on 277 of 9,428
segments of the Nishi-Shinjuku fixture (max 0.98 m on a 2.75 m segment).
Note that switching the declaration alone fixes nothing: the sibling rule
``road.geometry.parampoly3.normalized_range`` applies the *same* 1 mm test
over ``[0, 1]``.  What both rules really require is that ``@length`` be the
true arc length of the emitted curve.

**Defect 2 -- the emitted curve re-approximated the fitted one.**  The split
points were a uniform grid independent of the B-spline's knots, and the
Hermite construction discarded second derivatives, so the emitted chain was a
fresh approximation layered on top of the fit and lost the C2 continuity the
fit had.  Emitting one cubic per knot span removes both: inside a knot span a
cubic B-spline *is* a cubic polynomial.

Each test asserts the invariant and, where the old behaviour is still
reachable through ``knot_aligned = False``, pins the failure it used to show.
"""

import dataclasses

import numpy as np
import pytest
from scipy.integrate import quad

from autoware_lanelet2_to_opendrive import config as config_module
from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.conversion_config import ParamPoly3Config
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    PARAM_RANGE_NORMALIZED,
    ParamPoly3,
    end_param,
    param_for_offset,
    param_poly3_arc_length,
    param_poly3_speed,
)
from autoware_lanelet2_to_opendrive.spline import Splines

#: The tolerance both ASAM paramPoly3 length rules use.
ASAM_LENGTH_TOLERANCE = 1e-3

#: The Nishi-Shinjuku fixture's own segment target -- the configuration under
#: which the 277 ``arclength_range`` findings were measured.
MAP_CONFIG = ParamPoly3Config(default_segment_length=3.0, max_segments=50)


@pytest.fixture
def legacy_emission(monkeypatch):
    """Restore the pre-fix uniform-grid Hermite emission for one test."""
    patched = dataclasses.replace(
        DEFAULT_CONFIG,
        parampoly3=dataclasses.replace(DEFAULT_CONFIG.parampoly3, knot_aligned=False),
    )
    monkeypatch.setattr(config_module, "DEFAULT_CONFIG", patched)
    return patched


def _winding_spline() -> Splines:
    """A spline with enough curvature to expose the Hermite re-approximation."""
    t = np.linspace(0.0, 1.0, 80)
    points = np.column_stack(
        [
            60.0 * t + 4.0 * np.sin(9.0 * t),
            18.0 * np.sin(5.0 * t) + 3.0 * np.cos(17.0 * t),
            np.zeros_like(t),
        ]
    )
    return Splines(points, num_control_points=24)


def _hairpin_spline() -> Splines:
    """A tight hairpin: the worst case for a chord-like cubic Hermite fit."""
    theta = np.linspace(0.0, np.pi, 40)
    radius = 2.5
    points = np.column_stack(
        [radius * np.cos(theta), radius * np.sin(theta), np.zeros(len(theta))]
    )
    return Splines(points, num_control_points=8)


def _sample_world(geom: ParamPoly3, p: float) -> tuple:
    u = geom.aU + geom.bU * p + geom.cU * p**2 + geom.dU * p**3
    v = geom.aV + geom.bV * p + geom.cV * p**2 + geom.dV * p**3
    cos_h, sin_h = np.cos(geom.hdg), np.sin(geom.hdg)
    return (geom.x + u * cos_h - v * sin_h, geom.y + u * sin_h + v * cos_h)


def _kappa(geom: ParamPoly3, p: float) -> float:
    du = geom.bU + 2.0 * geom.cU * p + 3.0 * geom.dU * p * p
    dv = geom.bV + 2.0 * geom.cV * p + 3.0 * geom.dV * p * p
    ddu = 2.0 * geom.cU + 6.0 * geom.dU * p
    ddv = 2.0 * geom.cV + 6.0 * geom.dV * p
    speed = (du * du + dv * dv) ** 1.5
    if speed < 1e-18:
        return 0.0
    return float((du * ddv - dv * ddu) / speed)


def _asam_length_error(geom: ParamPoly3) -> float:
    """Reproduce the ASAM checker: |quad(|(u', v')|, 0, p_end) - @length|."""
    coeffs = geom.coefficients()
    integral, _ = quad(lambda p: param_poly3_speed(coeffs, p), 0.0, end_param(geom))
    return abs(float(integral) - geom.length)


# ---------------------------------------------------------------------------
# Defect 1: the declared parameter range and @length must be consistent
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("make_spline", [_winding_spline, _hairpin_spline])
def test_declared_length_is_the_true_arc_length(make_spline):
    """Every emitted segment survives the ASAM parampoly3 length rules."""
    segments = ParamPoly3.from_spline(make_spline(), config=ParamPoly3Config())

    assert segments
    for geom in segments:
        assert _asam_length_error(geom) < ASAM_LENGTH_TOLERANCE


def test_prange_is_normalized_and_p_runs_over_unit_interval():
    """``pRange`` names the range the coefficients are actually written in."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    assert {g.pRange for g in segments} == {PARAM_RANGE_NORMALIZED}
    for geom in segments:
        assert end_param(geom) == 1.0
        # p = 1 lands on the piece end, which is where the next piece starts.
        assert geom.to_xml().find("paramPoly3").get("pRange") == PARAM_RANGE_NORMALIZED


def test_legacy_uniform_emission_violates_the_arclength_declaration(legacy_emission):
    """Pins the defect: the old path declares arcLength but is not unit-speed."""
    segments = ParamPoly3.from_spline(_hairpin_spline(), config=MAP_CONFIG)

    assert {g.pRange for g in segments} == {"arcLength"}
    worst = max(_asam_length_error(g) for g in segments)
    assert worst > ASAM_LENGTH_TOLERANCE, (
        "the legacy path is expected to violate "
        "road.geometry.parampoly3.arclength_range"
    )


def test_param_for_offset_inverts_travelled_distance():
    """Sampling by travelled distance must not fall back to a linear guess."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())
    geom = max(segments, key=lambda g: g.length)

    for fraction in (0.0, 0.1, 0.37, 0.5, 0.82, 1.0):
        ds = fraction * geom.length
        p = param_for_offset(geom, ds)
        assert 0.0 <= p <= 1.0
        travelled = param_poly3_arc_length(geom.coefficients(), p)
        assert travelled == pytest.approx(ds, abs=1e-6)

    # A naive ds / length guess is measurably wrong on at least one segment,
    # which is why the inversion exists.
    naive_worst = max(
        abs(
            param_poly3_arc_length(g.coefficients(), 0.5 * g.length / g.length)
            - 0.5 * g.length
        )
        for g in segments
    )
    assert naive_worst > 1e-6


def test_param_for_offset_is_identity_for_arclength_geometries():
    """Straight synthesised connectors keep ``p == s`` semantics."""
    geom = ParamPoly3(
        s=0.0, x=0.0, y=0.0, hdg=0.0, length=2.0, bU=1.0, pRange="arcLength"
    )
    assert param_for_offset(geom, 1.25) == pytest.approx(1.25)
    assert end_param(geom) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# Defect 2: the emitted curve must *be* the fitted curve
# ---------------------------------------------------------------------------


def test_emitted_curve_reproduces_the_fitted_spline():
    """Knot-aligned pieces reproduce the B-spline to machine precision."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    worst = 0.0
    for geom in segments:
        for p in np.linspace(0.0, 1.0, 17):
            wx, wy = _sample_world(geom, p)
            s_closest = spline.find_closest_s(np.array([wx, wy]))
            ref = spline.evaluate(s_closest)
            worst = max(worst, float(np.hypot(ref[0] - wx, ref[1] - wy)))
    assert worst < 1e-6, f"emitted curve deviates from the fit by {worst} m"


def test_legacy_uniform_emission_deviates_from_the_fitted_spline(legacy_emission):
    """Pins the defect: the Hermite re-approximation adds its own error."""
    spline = _hairpin_spline()
    segments = ParamPoly3.from_spline(spline, config=MAP_CONFIG)

    worst = 0.0
    for geom in segments:
        for ds in np.linspace(0.0, geom.length, 17):
            wx, wy = _sample_world(geom, ds)
            s_closest = spline.find_closest_s(np.array([wx, wy]))
            ref = spline.evaluate(s_closest)
            worst = max(worst, float(np.hypot(ref[0] - wx, ref[1] - wy)))
    assert worst > 1e-3, "the legacy path is expected to re-approximate the fit"


def test_no_curvature_jump_between_consecutive_pieces():
    """The fit is C2; knot-aligned emission keeps it (no kappa step at seams)."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    assert len(segments) > 2
    for before, after in zip(segments, segments[1:]):
        # Position continuity.
        end_xy = _sample_world(before, 1.0)
        start_xy = _sample_world(after, 0.0)
        assert end_xy == pytest.approx(start_xy, abs=1e-9)
        # Curvature continuity.
        assert _kappa(before, 1.0) == pytest.approx(_kappa(after, 0.0), abs=1e-6)


def test_legacy_uniform_emission_jumps_in_curvature(legacy_emission):
    """Pins the defect: independent per-window fits break C2 at every seam."""
    segments = ParamPoly3.from_spline(_hairpin_spline(), config=MAP_CONFIG)

    worst = max(
        abs(_kappa(before, before.length) - _kappa(after, 0.0))
        for before, after in zip(segments, segments[1:])
    )
    assert worst > 1e-4, "the legacy path is expected to jump in curvature"


def test_pieces_are_cut_at_the_spline_breakpoints():
    """One emitted piece per knot span -- subdivision never crosses a knot."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    assert len(segments) == len(spline.breakpoints()) - 1

    starts = [spline.param_at_arc_length(0.0)]
    cumulative = 0.0
    for geom in segments[:-1]:
        cumulative += geom.length
        starts.append(spline.param_at_arc_length(cumulative))
    expected = spline.breakpoints()[:-1]
    assert np.allclose(starts, expected, atol=1e-3)


def test_s_offsets_are_contiguous_and_sum_to_the_road_length():
    """``s_i + length_i == s_{i+1}`` exactly; the sum is the reference length."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    assert segments[0].s == pytest.approx(0.0)
    for before, after in zip(segments, segments[1:]):
        assert after.s == pytest.approx(before.s + before.length, abs=1e-12)
    total = segments[-1].s + segments[-1].length
    assert total == pytest.approx(spline.total_length, rel=1e-4)


def test_planview_endpoints_land_on_the_spline_endpoints():
    """The rendered chain still starts and ends exactly where the fit does."""
    spline = _winding_spline()
    segments = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    start = spline.evaluate(0.0)
    end = spline.evaluate(spline.total_length)
    assert _sample_world(segments[0], 0.0) == pytest.approx(
        (start[0], start[1]), abs=1e-6
    )
    assert _sample_world(segments[-1], end_param(segments[-1])) == pytest.approx(
        (end[0], end[1]), abs=1e-6
    )


def test_knot_span_max_length_subdivides_without_losing_exactness(monkeypatch):
    """A length cap only cuts *inside* a span, so the pieces stay exact."""
    spline = _winding_spline()
    uncapped = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    patched = dataclasses.replace(
        DEFAULT_CONFIG,
        parampoly3=dataclasses.replace(
            DEFAULT_CONFIG.parampoly3, knot_span_max_length=1.0
        ),
    )
    monkeypatch.setattr(config_module, "DEFAULT_CONFIG", patched)
    capped = ParamPoly3.from_spline(spline, config=ParamPoly3Config())

    assert len(capped) > len(uncapped)
    assert all(g.length <= 1.5 for g in capped)
    for geom in capped:
        assert _asam_length_error(geom) < ASAM_LENGTH_TOLERANCE
        for p in np.linspace(0.0, 1.0, 9):
            wx, wy = _sample_world(geom, p)
            ref = spline.evaluate(spline.find_closest_s(np.array([wx, wy])))
            assert float(np.hypot(ref[0] - wx, ref[1] - wy)) < 1e-6
