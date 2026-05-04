"""Tests for <lane><border> emission on asymmetric lanelets (Issue #440)."""

from autoware_lanelet2_to_opendrive.config import (
    DEFAULT_CONFIG,
    LaneBorderConstants,
)


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
