"""Tests for <lane><border> emission on asymmetric lanelets (Issue #440)."""

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.centerline import (
    _max_relative_asymmetry,
    _max_width_fit_residual,
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
