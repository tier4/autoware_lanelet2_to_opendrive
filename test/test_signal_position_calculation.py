"""Tests for signal position calculation from Lanelet2 geometry."""

import numpy as np
from unittest.mock import Mock
from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers
from autoware_lanelet2_to_opendrive.spline import Splines


def test_calculate_signal_position_with_valid_geometry():
    """Test signal position calculation with valid traffic light geometry."""
    # Create a mock traffic light with geometry
    mock_traffic_light = Mock()

    # Create a mock position point
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0

    # Create a mock linestring with the point
    mock_linestring = [mock_point]
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    # Create a simple spline for testing
    # Create a straight line along x-axis from (0, 0, 0) to (20, 0, 0)
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [15.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],
        ]
    )
    spline = Splines(points)

    # Create spline cache
    road_spline_cache = {0: spline}

    # Calculate signal position
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        road_spline_cache=road_spline_cache,
    )

    # For a straight line along x-axis, the signal at (10, 5, 2) should be:
    # - s ≈ 10.0 (x-coordinate along the spline)
    # - t ≈ 5.0 (y-offset from the spline, positive = left)
    assert abs(s - 10.0) < 1.0, f"Expected s ≈ 10.0, got {s}"
    assert abs(t - 5.0) < 1.0, f"Expected t ≈ 5.0, got {t}"


def test_calculate_signal_position_with_no_geometry():
    """Test signal position calculation when traffic light has no geometry."""
    mock_traffic_light = Mock()
    mock_traffic_light.trafficLights = []
    mock_traffic_light.id = 1

    road_spline_cache = {}

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        road_spline_cache=road_spline_cache,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_with_empty_linestring():
    """Test signal position calculation when linestring is empty."""
    mock_traffic_light = Mock()
    mock_traffic_light.trafficLights = [[]]  # Empty linestring
    mock_traffic_light.id = 1

    road_spline_cache = {}

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        road_spline_cache=road_spline_cache,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_without_spline():
    """Test signal position calculation when road has no spline in cache."""
    mock_traffic_light = Mock()
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0
    mock_linestring = [mock_point]
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    # Empty cache
    road_spline_cache = {}

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        road_spline_cache=road_spline_cache,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_negative_t():
    """Test signal position calculation with negative t (right side)."""
    # Create a mock traffic light with geometry
    mock_traffic_light = Mock()
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = -3.0  # Negative y means right side of the road
    mock_point.z = 2.0
    mock_linestring = [mock_point]
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    # Create a simple spline for testing (along x-axis)
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [15.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],
        ]
    )
    spline = Splines(points)

    # Create spline cache
    road_spline_cache = {0: spline}

    # Calculate signal position
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        road_spline_cache=road_spline_cache,
    )

    # The signal at (10, -3, 2) should have:
    # - s ≈ 10.0 (x-coordinate along the spline)
    # - t ≈ -3.0 (y-offset from the spline, negative = right)
    assert abs(s - 10.0) < 1.0, f"Expected s ≈ 10.0, got {s}"
    assert abs(t - (-3.0)) < 1.0, f"Expected t ≈ -3.0, got {t}"


def test_build_road_spline_cache_empty_mapping():
    """Test building road spline cache with empty mapping."""
    mock_lanelet_map = Mock()
    empty_mapping = Mock()
    empty_mapping.road_to_lanelets = {}

    cache = SignalsAndControllers._build_road_spline_cache(
        mock_lanelet_map, empty_mapping
    )

    assert len(cache) == 0
    assert isinstance(cache, dict)
