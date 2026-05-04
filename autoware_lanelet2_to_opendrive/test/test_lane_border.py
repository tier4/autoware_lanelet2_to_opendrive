"""Tests for ``<lane><border>`` emission on asymmetric lanelets (Issue #440).

These tests use lightweight mock lanelets — the host environment cannot
build ``lanelet2-python-api-for-autoware`` against system Boost, so
spinning up real ``lanelet2.core.Lanelet`` instances in unit tests is not
viable. Mocks expose ``.id``, ``.leftBound``, ``.rightBound``, each
iterable of objects with ``.x``, ``.y``, ``.z`` — sufficient for
``extract_points_3d``.

The end-to-end coverage on the real ``nishishinjuku.osm`` map comes from
the existing ``test_lht_lane_widths_*`` integration tests in
``test_integration_traffic_rule.py``, updated alongside this PR to
accept either ``<width>`` or ``<border>`` per lane.
"""

from __future__ import annotations

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.centerline import (
    _closest_point_on_polyline_2d,
    _fit_signed_t_spline,
    _max_outer_bound_deviation,
    compute_lane_outer_polynomial,
    estimate_lanelet_width_with_reference_line,
)
from autoware_lanelet2_to_opendrive.config import (
    DEFAULT_CONFIG,
    LaneBorderConstants,
)
from autoware_lanelet2_to_opendrive.conversion_config import (
    WidthEstimationConfig,
    WidthReference,
)
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LanePolynomial
from autoware_lanelet2_to_opendrive.spline import Splines


# ---------------------------------------------------------------------------
# Mocks
# ---------------------------------------------------------------------------


class _MockPoint:
    """Stand-in for ``lanelet2.core.Point3d`` exposing only ``.x .y .z``."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


class _MockLanelet:
    """Stand-in for ``lanelet2.core.Lanelet`` exposing only the surface used
    by ``compute_lane_outer_polynomial`` and its helpers."""

    def __init__(self, left, right, lanelet_id: int = 1) -> None:
        self.leftBound = left
        self.rightBound = right
        self.id = lanelet_id


def _make_straight_ref_spline(length: float = 10.0, n: int = 21) -> Splines:
    """Reference line: straight along +x at y=0, z=0."""
    pts = np.array([[i * length / (n - 1), 0.0, 0.0] for i in range(n)])
    return Splines(pts)


def _symmetric_straight_lanelet(width: float = 2.0) -> _MockLanelet:
    """Straight 10 m lanelet: leftBound at y=+w/2, rightBound at y=-w/2."""
    half = width / 2.0
    xs = np.linspace(0.0, 10.0, 21)
    left = [_MockPoint(x, +half) for x in xs]
    right = [_MockPoint(x, -half) for x in xs]
    return _MockLanelet(left, right)


def _symmetric_curved_lanelet(width: float = 2.0) -> _MockLanelet:
    """Concentric-arc symmetric lanelet (the case PR #458 false-tripped on).

    A quarter-circle around centre ``(0, R)`` with ``R = 20`` m. For RHT
    convention the lanelet turns counter-clockwise, so ``leftBound`` is the
    *inner* arc (smaller radius) and ``rightBound`` is the *outer* arc
    (larger radius); both share the same centre, perpendicular distance is
    constant ``= width``.
    """
    R = 20.0
    half = width / 2.0
    inner_r = R - half
    outer_r = R + half
    angles = np.linspace(0.0, np.pi / 2.0, 41)
    left = [_MockPoint(inner_r * np.sin(a), R - inner_r * np.cos(a)) for a in angles]
    right = [_MockPoint(outer_r * np.sin(a), R - outer_r * np.cos(a)) for a in angles]
    return _MockLanelet(left, right)


def _bulged_right_lanelet() -> _MockLanelet:
    """Left bound straight at y=+1.0; right bound bulges into y∈[-2, -1].

    The geometry is smooth, so the absolute deviation the metric reports is
    small — but non-zero. Tests using this mock force ``<border>`` via a
    tight ``deviation_tolerance`` override; constructing a synthetic
    polyline whose deviation reliably exceeds the production default of
    0.30 m without contrived artefacts is awkward, and the production
    threshold itself is exercised end-to-end by the
    ``test_lht_lane_widths_*`` integration suite on real OSM data.
    """
    xs = np.linspace(0.0, 10.0, 21)
    bulge_y = -1.0 - np.sin(np.pi * xs / 10.0)
    left = [_MockPoint(x, 1.0) for x in xs]
    right = [_MockPoint(x, y) for x, y in zip(xs, bulge_y)]
    return _MockLanelet(left, right)


def _bulged_left_lanelet() -> _MockLanelet:
    """Mirror of ``_bulged_right_lanelet`` for the LHT outer-bound case."""
    xs = np.linspace(0.0, 10.0, 21)
    bulge_y = +1.0 + np.sin(np.pi * xs / 10.0)
    left = [_MockPoint(x, y) for x, y in zip(xs, bulge_y)]
    right = [_MockPoint(x, -1.0) for x in xs]
    return _MockLanelet(left, right)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


def test_lane_border_constants_default():
    """``LaneBorderConstants`` exposes the documented default tolerance."""
    cfg = LaneBorderConstants()
    assert cfg.outer_bound_deviation_tolerance == 0.30


def test_default_config_exposes_lane_border():
    """``DEFAULT_CONFIG.lane_border`` is wired into ``ConversionConstants``."""
    assert isinstance(DEFAULT_CONFIG.lane_border, LaneBorderConstants)
    assert DEFAULT_CONFIG.lane_border.outer_bound_deviation_tolerance == 0.30


# ---------------------------------------------------------------------------
# LanePolynomial dataclass
# ---------------------------------------------------------------------------


def test_lane_polynomial_width_kind():
    """A ``width``-kind polynomial holds segment tuples for ``<lane><width>``."""
    poly = LanePolynomial(
        kind="width",
        segments=[(0.0, 3.5, 0.0, 0.0, 0.0)],
        total_length=10.0,
    )
    assert poly.kind == "width"
    assert poly.segments[0] == (0.0, 3.5, 0.0, 0.0, 0.0)
    assert poly.total_length == 10.0


def test_lane_polynomial_border_kind():
    """A ``border``-kind polynomial holds segment tuples for ``<lane><border>``."""
    poly = LanePolynomial(
        kind="border",
        segments=[(0.0, -3.5, 0.1, 0.0, 0.0)],
        total_length=10.0,
    )
    assert poly.kind == "border"
    assert poly.segments[0][1] == -3.5  # negative t for the right side


def test_lane_polynomial_kind_validation():
    """``LanePolynomial.kind`` only accepts ``"width"`` or ``"border"``."""
    with pytest.raises(ValueError):
        LanePolynomial(kind="invalid", segments=[], total_length=1.0)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Helper: closest point on polyline
# ---------------------------------------------------------------------------


def test_closest_point_on_polyline_perpendicular_foot():
    """Closest point above a horizontal segment is the perpendicular foot."""
    polyline = np.array([[0.0, 0.0], [10.0, 0.0]])
    p = np.array([3.0, 4.0])
    q = _closest_point_on_polyline_2d(p, polyline)
    assert q == pytest.approx(np.array([3.0, 0.0]), abs=1e-9)


def test_closest_point_on_polyline_clamps_at_vertex():
    """A query past the segment end clamps to the nearest endpoint."""
    polyline = np.array([[0.0, 0.0], [10.0, 0.0]])
    p = np.array([15.0, 1.0])
    q = _closest_point_on_polyline_2d(p, polyline)
    assert q == pytest.approx(np.array([10.0, 0.0]), abs=1e-9)


# ---------------------------------------------------------------------------
# Detection metric
# ---------------------------------------------------------------------------


def _config(num_samples: int = 41) -> WidthEstimationConfig:
    return WidthEstimationConfig(
        num_samples=num_samples, reference=WidthReference.LEFT_BOUND
    )


def test_max_outer_bound_deviation_symmetric_straight_is_small():
    """A perfectly symmetric straight lanelet has near-zero deviation."""
    lanelet = _symmetric_straight_lanelet()
    ref_line = _make_straight_ref_spline()
    config = _config()
    width_adapter = estimate_lanelet_width_with_reference_line(
        lanelet, ref_line, config
    )
    deviation = _max_outer_bound_deviation(
        lanelet, ref_line, width_adapter, config, rule="RHT"
    )
    assert deviation < 0.01  # 1 cm


def test_max_outer_bound_deviation_symmetric_curved_is_small():
    """Concentric-arc symmetric lanelet stays below tolerance.

    This is the explicit guard against the false positive that closed
    PR #458: equal-``t`` sampling on two unequal-length curves produced
    large "asymmetry" on perfectly symmetric curved lanes. The
    perpendicular-projection metric is curvature-aware and must NOT fire.
    """
    lanelet = _symmetric_curved_lanelet()
    # Reference line follows the lanelet's left bound for RHT (anchor).
    left_pts = np.array([[p.x, p.y] for p in lanelet.leftBound])
    ref_line = Splines(left_pts)
    config = _config()
    width_adapter = estimate_lanelet_width_with_reference_line(
        lanelet, ref_line, config
    )
    deviation = _max_outer_bound_deviation(
        lanelet, ref_line, width_adapter, config, rule="RHT"
    )
    assert deviation < DEFAULT_CONFIG.lane_border.outer_bound_deviation_tolerance


def test_max_outer_bound_deviation_one_sided_bulge_is_nonzero():
    """Right-bulged lanelet has measurably non-zero deviation.

    Synthetic smooth bulges produce small absolute deviations (matched-arc-
    length sampling roughly aligns with perpendicular at sample points on
    smooth bounds); the value is positive but typically below the
    production 0.30 m default. The metric's correctness is asserted here
    by lower-bounding it strictly above the symmetric-case noise floor.
    Real OSM lanelets that need ``<border>`` produce deviations well
    above 0.30 m, exercised end-to-end by the ``test_lht_lane_widths_*``
    integration suite.
    """
    lanelet = _bulged_right_lanelet()
    left_pts = np.array([[p.x, p.y] for p in lanelet.leftBound])
    ref_line = Splines(left_pts)
    config = _config()
    width_adapter = estimate_lanelet_width_with_reference_line(
        lanelet, ref_line, config
    )
    deviation = _max_outer_bound_deviation(
        lanelet, ref_line, width_adapter, config, rule="RHT"
    )
    assert deviation > 1e-4
    # Symmetric-straight noise floor for cross-check:
    sym = _symmetric_straight_lanelet()
    sym_dev = _max_outer_bound_deviation(
        sym,
        ref_line,
        estimate_lanelet_width_with_reference_line(sym, ref_line, config),
        config,
        rule="RHT",
    )
    assert deviation > sym_dev


def test_max_outer_bound_deviation_center_line_reference_returns_zero():
    """``CENTER_LINE`` reference disables the perpendicular trigger."""
    lanelet = _bulged_right_lanelet()
    ref_line = _make_straight_ref_spline()
    config = WidthEstimationConfig(num_samples=21, reference=WidthReference.CENTER_LINE)
    width_adapter = estimate_lanelet_width_with_reference_line(
        lanelet, ref_line, config
    )
    deviation = _max_outer_bound_deviation(
        lanelet, ref_line, width_adapter, config, rule="RHT"
    )
    assert deviation == 0.0


# ---------------------------------------------------------------------------
# Border t-spline fit
# ---------------------------------------------------------------------------


def test_fit_signed_t_spline_right_bound_yields_negative_t():
    """A right bound at y=-1 along a +x ref line yields signed t ≈ -1."""
    ref_line = _make_straight_ref_spline()
    bound_pts = np.array([[x, -1.0, 0.0] for x in np.linspace(0.0, 10.0, 11)])
    config = _config()
    spline = _fit_signed_t_spline(bound_pts, ref_line, config)
    for s in (0.0, 2.5, 5.0, 7.5, 10.0):
        assert spline.evaluate(s) == pytest.approx(-1.0, abs=5e-3)


def test_fit_signed_t_spline_left_bound_yields_positive_t():
    """A left bound at y=+1 along a +x ref line yields signed t ≈ +1."""
    ref_line = _make_straight_ref_spline()
    bound_pts = np.array([[x, +1.0, 0.0] for x in np.linspace(0.0, 10.0, 11)])
    config = _config()
    spline = _fit_signed_t_spline(bound_pts, ref_line, config)
    for s in (0.0, 2.5, 5.0, 7.5, 10.0):
        assert spline.evaluate(s) == pytest.approx(+1.0, abs=5e-3)


def test_fit_signed_t_spline_first_segment_starts_at_zero():
    """First segment ``sOffset`` is ``0.0`` (matches ``<width>`` convention)."""
    ref_line = _make_straight_ref_spline()
    bound_pts = np.array([[x, -1.0, 0.0] for x in np.linspace(0.0, 10.0, 11)])
    config = _config()
    spline = _fit_signed_t_spline(bound_pts, ref_line, config)
    segments = spline.get_segments()
    assert segments
    assert segments[0][0] == 0.0


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def test_compute_lane_outer_polynomial_symmetric_returns_width():
    """A symmetric lanelet returns ``kind="width"`` with segments and length
    matching ``estimate_lanelet_width_with_reference_line``."""
    lanelet = _symmetric_straight_lanelet()
    left_pts = np.array([[p.x, p.y] for p in lanelet.leftBound])
    ref_line = Splines(left_pts)
    config = _config()

    poly = compute_lane_outer_polynomial(lanelet, ref_line, config, rule="RHT")
    assert poly.kind == "width"

    legacy = estimate_lanelet_width_with_reference_line(lanelet, ref_line, config)
    assert poly.segments == legacy.get_polynomial_segments()
    assert poly.total_length == pytest.approx(legacy.total_length)


def test_compute_lane_outer_polynomial_curved_symmetric_stays_width():
    """The curved-symmetric guard against PR #458's false positive holds end-to-end."""
    lanelet = _symmetric_curved_lanelet()
    left_pts = np.array([[p.x, p.y] for p in lanelet.leftBound])
    ref_line = Splines(left_pts)
    config = _config()
    poly = compute_lane_outer_polynomial(lanelet, ref_line, config, rule="RHT")
    assert poly.kind == "width"


def test_compute_lane_outer_polynomial_bulged_right_returns_border():
    """With a tight tolerance override, a right-bulged lanelet emits border.

    Forces the routing through the ``<border>`` branch by overriding the
    deviation tolerance to near-zero so the synthetic mock's small but
    non-zero deviation trips the trigger. Verifies the border fit emits
    a negative-t leading segment for an RHT outer bound.
    """
    lanelet = _bulged_right_lanelet()
    left_pts = np.array([[p.x, p.y] for p in lanelet.leftBound])
    ref_line = Splines(left_pts)
    config = _config()
    poly = compute_lane_outer_polynomial(
        lanelet, ref_line, config, rule="RHT", deviation_tolerance=1e-6
    )
    assert poly.kind == "border"
    assert poly.segments
    s0, a0, _b, _c, _d = poly.segments[0]
    assert s0 == 0.0
    # First-segment ``a`` is signed t at s=0; right outer bound → negative.
    assert a0 < 0.0


def test_compute_lane_outer_polynomial_bulged_left_lht_returns_border():
    """LHT mirror with tight-tolerance override: outer = leftBound, a > 0."""
    lanelet = _bulged_left_lanelet()
    right_pts = np.array([[p.x, p.y] for p in lanelet.rightBound])
    ref_line = Splines(right_pts)
    config = WidthEstimationConfig(num_samples=41, reference=WidthReference.RIGHT_BOUND)
    poly = compute_lane_outer_polynomial(
        lanelet, ref_line, config, rule="LHT", deviation_tolerance=1e-6
    )
    assert poly.kind == "border"
    s0, a0, _b, _c, _d = poly.segments[0]
    assert s0 == 0.0
    assert a0 > 0.0


def test_compute_lane_outer_polynomial_threshold_disable_keeps_width():
    """Raising the tolerance high enough leaves even a bulged lanelet on width."""
    lanelet = _bulged_right_lanelet()
    left_pts = np.array([[p.x, p.y] for p in lanelet.leftBound])
    ref_line = Splines(left_pts)
    config = _config()
    poly = compute_lane_outer_polynomial(
        lanelet, ref_line, config, rule="RHT", deviation_tolerance=10.0
    )
    assert poly.kind == "width"


def test_compute_lane_outer_polynomial_invalid_rule_raises():
    """Unknown ``rule`` values are rejected up front."""
    lanelet = _symmetric_straight_lanelet()
    ref_line = _make_straight_ref_spline()
    config = _config()
    with pytest.raises(ValueError):
        compute_lane_outer_polynomial(lanelet, ref_line, config, rule="XYZ")
