"""Tests for <lane><border> emission on asymmetric lanelets (Issue #440)."""

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.centerline import (
    _fit_signed_t_spline,
    _max_relative_asymmetry,
    _max_width_fit_residual,
    compute_lane_outer_polynomial,
    estimate_lanelet_width_as_spline,
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


class _MockPoint:
    def __init__(self, x: float, y: float, z: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.z = z


class _MockLanelet:
    def __init__(self, left, right, lanelet_id: int = 1) -> None:
        self.leftBound = left
        self.rightBound = right
        self.id = lanelet_id


def _symmetric_lanelet() -> _MockLanelet:
    """Straight 10 m lanelet, leftBound at y=+1, rightBound at y=-1."""
    left = [_MockPoint(x, 1.0) for x in np.linspace(0, 10, 11)]
    right = [_MockPoint(x, -1.0) for x in np.linspace(0, 10, 11)]
    return _MockLanelet(left, right)


def _bulged_right_lanelet() -> _MockLanelet:
    """Left straight at y=+1; right bulges from y=-1 to y=-2 to y=-1."""
    left = [_MockPoint(x, 1.0) for x in np.linspace(0, 10, 11)]
    bulge = [-1.0, -1.2, -1.5, -1.8, -2.0, -2.0, -2.0, -1.8, -1.5, -1.2, -1.0]
    right = [_MockPoint(x, y) for x, y in zip(np.linspace(0, 10, 11), bulge)]
    return _MockLanelet(left, right)


def test_lane_border_constants_defaults():
    """LaneBorderConstants must expose the documented default tolerances."""
    cfg = LaneBorderConstants()
    assert cfg.asymmetry_tolerance == 0.10
    assert cfg.width_residual_tolerance == 0.30


def test_default_config_exposes_lane_border():
    """DEFAULT_CONFIG must expose lane_border with the same defaults."""
    assert isinstance(DEFAULT_CONFIG.lane_border, LaneBorderConstants)
    assert DEFAULT_CONFIG.lane_border.asymmetry_tolerance == 0.10
    assert DEFAULT_CONFIG.lane_border.width_residual_tolerance == 0.30


def test_lane_polynomial_width_kind():
    """A width-kind polynomial holds (s,a,b,c,d) tuples for <lane><width>."""
    poly = LanePolynomial(
        kind="width",
        segments=[(0.0, 3.5, 0.0, 0.0, 0.0)],
        total_length=10.0,
    )
    assert poly.kind == "width"
    assert poly.segments[0] == (0.0, 3.5, 0.0, 0.0, 0.0)
    assert poly.total_length == 10.0


def test_lane_polynomial_border_kind():
    """A border-kind polynomial holds (s,a,b,c,d) tuples for <lane><border>."""
    poly = LanePolynomial(
        kind="border",
        segments=[(0.0, -3.5, 0.1, 0.0, 0.0)],
        total_length=10.0,
    )
    assert poly.kind == "border"
    assert poly.segments[0][1] == -3.5  # negative t for right side


def test_lane_polynomial_kind_validation():
    """LanePolynomial only accepts kind in {'width', 'border'}."""
    with pytest.raises((ValueError, TypeError)):
        LanePolynomial(kind="invalid", segments=[], total_length=1.0)  # type: ignore


def test_max_relative_asymmetry_symmetric_is_zero():
    """A symmetric lanelet has near-zero left/right asymmetry."""
    lanelet = _symmetric_lanelet()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.LEFT_BOUND)
    ratio = _max_relative_asymmetry(lanelet, config)
    assert ratio < 1e-6


def test_max_relative_asymmetry_bulged_right_trips_threshold():
    """Right-bulged lanelet has relative asymmetry well above 10%."""
    lanelet = _bulged_right_lanelet()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.LEFT_BOUND)
    ratio = _max_relative_asymmetry(lanelet, config)
    assert ratio > 0.10  # well above DEFAULT_CONFIG.lane_border.asymmetry_tolerance


def test_max_width_fit_residual_symmetric_is_small():
    """A symmetric straight lanelet has residual far below 1 cm."""
    lanelet = _symmetric_lanelet()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.LEFT_BOUND)
    width_adapter = estimate_lanelet_width_as_spline(lanelet, config)
    residual = _max_width_fit_residual(lanelet, width_adapter, config)
    assert residual < 0.01


def test_max_width_fit_residual_s_shape_is_large():
    """S-shape width (non-monotonic) trips the residual threshold."""
    # Width oscillates: 2.0 -> 3.5 -> 2.0 -> 3.5 -> 2.0 in 5 stations.
    # A single cubic cannot fit two interior extrema, so residual > 0.30 m.
    left = [_MockPoint(x, 1.75) for x in np.linspace(0, 10, 21)]
    right_y = []
    for x in np.linspace(0, 10, 21):
        # 4 extrema profile: -0.25, -1.75, -0.25, -1.75, -0.25
        phase = (x / 10.0) * 4.0 * np.pi
        right_y.append(-1.0 + 0.75 * np.cos(phase))  # right_y in [-1.75, -0.25]
    right = [_MockPoint(x, y) for x, y in zip(np.linspace(0, 10, 21), right_y)]
    lanelet = _MockLanelet(left, right, lanelet_id=2)
    config = WidthEstimationConfig(num_samples=21, reference=WidthReference.LEFT_BOUND)
    width_adapter = estimate_lanelet_width_as_spline(lanelet, config)
    residual = _max_width_fit_residual(lanelet, width_adapter, config)
    assert residual > 0.30


def _make_straight_xy_spline(length: float = 10.0, n: int = 11) -> Splines:
    """Reference line: straight along +x axis, y=0, z=0."""
    pts = np.array([[x, 0.0, 0.0] for x in np.linspace(0, length, n)])
    return Splines(pts)


def test_fit_signed_t_spline_left_bound_positive_t():
    """Left bound at y=+1 along a +x reference line yields t = +1."""
    ref_line = _make_straight_xy_spline()
    bound_points = np.array([[x, 1.0, 0.0] for x in np.linspace(0, 10, 11)])
    adapter = _fit_signed_t_spline(bound_points, ref_line)
    # Sample a few s and confirm t ~ +1
    for s in (0.0, 2.5, 5.0, 7.5, 10.0):
        t = adapter.evaluate(s)
        assert abs(t - 1.0) < 1e-3


def test_fit_signed_t_spline_right_bound_negative_t():
    """Right bound at y=-1 along a +x reference line yields t = -1."""
    ref_line = _make_straight_xy_spline()
    bound_points = np.array([[x, -1.0, 0.0] for x in np.linspace(0, 10, 11)])
    adapter = _fit_signed_t_spline(bound_points, ref_line)
    for s in (0.0, 2.5, 5.0, 7.5, 10.0):
        t = adapter.evaluate(s)
        assert abs(t - (-1.0)) < 1e-3


def test_fit_signed_t_spline_returns_segments():
    """The returned adapter must expose get_segments() for cubic export."""
    ref_line = _make_straight_xy_spline()
    bound_points = np.array([[x, -1.0, 0.0] for x in np.linspace(0, 10, 11)])
    adapter = _fit_signed_t_spline(bound_points, ref_line)
    segments = adapter.get_segments()
    assert len(segments) >= 1
    s, a, b, c, d = segments[0]
    assert s == 0.0
    assert abs(a - (-1.0)) < 1e-3  # y intercept = t value at s=0


def test_compute_lane_outer_polynomial_symmetric_is_width():
    """Symmetric lanelet returns kind='width' and segments equal to the
    existing width-only path's output (numerical regression guard)."""
    lanelet = _symmetric_lanelet()
    ref_line = _make_straight_xy_spline()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.LEFT_BOUND)
    poly = compute_lane_outer_polynomial(lanelet, ref_line, config, rule="RHT")
    assert poly.kind == "width"
    # Compare against the legacy adapter directly - bytes-identical
    # for symmetric input.
    legacy = estimate_lanelet_width_as_spline(lanelet, config)
    assert poly.segments == legacy.get_polynomial_segments()
    assert poly.total_length == legacy.total_length


def test_compute_lane_outer_polynomial_bulged_is_border():
    """Right-bulged lanelet trips the asymmetry threshold -> kind='border'."""
    lanelet = _bulged_right_lanelet()
    ref_line = _make_straight_xy_spline()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.LEFT_BOUND)
    poly = compute_lane_outer_polynomial(lanelet, ref_line, config, rule="RHT")
    assert poly.kind == "border"
    assert len(poly.segments) >= 1
    # First segment a-coefficient is signed t at s=0; right side -> negative.
    s0, a0, _, _, _ = poly.segments[0]
    assert s0 == 0.0
    assert a0 < 0.0


def test_compute_lane_outer_polynomial_lht_uses_left_bound_for_border():
    """For LHT rule, the outer bound is leftBound -> positive t at s=0."""
    # Use the bulged lanelet but interpret LHT (outer = leftBound).
    # Mirror the bulge so the LEFT bound bulges and right is straight.
    left_y = [1.0, 1.2, 1.5, 1.8, 2.0, 2.0, 2.0, 1.8, 1.5, 1.2, 1.0]
    left = [_MockPoint(x, y) for x, y in zip(np.linspace(0, 10, 11), left_y)]
    right = [_MockPoint(x, -1.0) for x in np.linspace(0, 10, 11)]
    lanelet = _MockLanelet(left, right, lanelet_id=3)
    ref_line = _make_straight_xy_spline()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.RIGHT_BOUND)
    poly = compute_lane_outer_polynomial(lanelet, ref_line, config, rule="LHT")
    assert poly.kind == "border"
    s0, a0, _, _, _ = poly.segments[0]
    assert s0 == 0.0
    assert a0 > 0.0  # left side -> positive t


def test_compute_lane_outer_polynomial_threshold_disable():
    """With both tolerances raised high, even an asymmetric lanelet stays width."""
    lanelet = _bulged_right_lanelet()
    ref_line = _make_straight_xy_spline()
    config = WidthEstimationConfig(num_samples=11, reference=WidthReference.LEFT_BOUND)
    poly = compute_lane_outer_polynomial(
        lanelet,
        ref_line,
        config,
        rule="RHT",
        asymmetry_tolerance=10.0,  # effectively disabled
        width_residual_tolerance=100.0,  # effectively disabled
    )
    assert poly.kind == "width"
