"""Tests for signal position calculation from Lanelet2 geometry."""

import numpy as np
from unittest.mock import Mock, patch
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

    # Mock the lanelet_map and road_lanelet_mapping
    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    # Mock lanelet objects
    mock_lanelet = Mock()
    mock_lanelet_map.laneletLayer.get.return_value = mock_lanelet

    # Mock ReferenceLine to return our test spline
    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_reference_line_class:
        mock_reference_line = Mock()
        mock_reference_line.centerline_2d = spline
        mock_reference_line_class.construct_from_lanelet_groups.return_value = (
            mock_reference_line
        )

        # Calculate signal position
        s, t = SignalsAndControllers._calculate_signal_position(
            traffic_light=mock_traffic_light,
            road_id=0,
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mock_road_lanelet_mapping,
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

    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        lanelet_map=mock_lanelet_map,
        road_lanelet_mapping=mock_road_lanelet_mapping,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_with_empty_linestring():
    """Test signal position calculation when linestring is empty."""
    mock_traffic_light = Mock()
    mock_traffic_light.trafficLights = [[]]  # Empty linestring
    mock_traffic_light.id = 1

    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        lanelet_map=mock_lanelet_map,
        road_lanelet_mapping=mock_road_lanelet_mapping,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_without_spline():
    """Test signal position calculation when road has no lanelets."""
    mock_traffic_light = Mock()
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0
    mock_linestring = [mock_point]
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    # Mock empty lanelet list for road
    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = []

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        lanelet_map=mock_lanelet_map,
        road_lanelet_mapping=mock_road_lanelet_mapping,
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

    # Mock the lanelet_map and road_lanelet_mapping
    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    # Mock lanelet objects
    mock_lanelet = Mock()
    mock_lanelet_map.laneletLayer.get.return_value = mock_lanelet

    # Mock ReferenceLine to return our test spline
    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_reference_line_class:
        mock_reference_line = Mock()
        mock_reference_line.centerline_2d = spline
        mock_reference_line_class.construct_from_lanelet_groups.return_value = (
            mock_reference_line
        )

        # Calculate signal position
        s, t = SignalsAndControllers._calculate_signal_position(
            traffic_light=mock_traffic_light,
            road_id=0,
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mock_road_lanelet_mapping,
        )

    # The signal at (10, -3, 2) should have:
    # - s ≈ 10.0 (x-coordinate along the spline)
    # - t ≈ -3.0 (y-offset from the spline, negative = right)
    assert abs(s - 10.0) < 1.0, f"Expected s ≈ 10.0, got {s}"
    assert abs(t - (-3.0)) < 1.0, f"Expected t ≈ -3.0, got {t}"


def test_calculate_signal_position_lanelet_not_found():
    """Test signal position calculation when lanelet cannot be retrieved."""
    mock_traffic_light = Mock()
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0
    mock_linestring = [mock_point]
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    # Mock lanelet_map that throws exception when getting lanelet
    mock_lanelet_map = Mock()
    mock_lanelet_map.laneletLayer.get.side_effect = Exception("Lanelet not found")

    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    # Should return default position (0.0, -4.0) when lanelets cannot be retrieved
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        road_id=0,
        lanelet_map=mock_lanelet_map,
        road_lanelet_mapping=mock_road_lanelet_mapping,
    )

    assert s == 0.0
    assert t == -4.0
