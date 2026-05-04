"""Tests for <lane><border> emission on asymmetric lanelets (Issue #440)."""

import pytest

from autoware_lanelet2_to_opendrive.config import (
    DEFAULT_CONFIG,
    LaneBorderConstants,
)
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LanePolynomial


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
