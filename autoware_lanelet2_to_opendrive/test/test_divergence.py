"""Tests for divergence/merge synthesis (issue #291)."""

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG


def test_geometry_constants_expose_divergence_thresholds():
    """Sanity gate and epsilon-floor live on GeometryConstants."""
    geom = DEFAULT_CONFIG.geometry
    assert geom.divergence_endpoint_tolerance == 0.5
    assert geom.divergence_min_segment_length == 0.01
