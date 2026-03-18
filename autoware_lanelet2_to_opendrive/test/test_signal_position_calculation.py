"""Tests for signal position calculation from Lanelet2 geometry."""

import math

import numpy as np
from unittest.mock import Mock, patch
from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers
from autoware_lanelet2_to_opendrive.spline import Splines


def test_calculate_signal_position_with_valid_geometry():
    """Test signal position calculation with valid traffic light geometry."""
    # Create a mock traffic light with geometry
    mock_traffic_light = Mock()
    mock_traffic_light.stopLine = None

    # Create a mock position point
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0

    # Create a mock linestring with the point
    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=1)
    mock_linestring.__getitem__ = Mock(return_value=mock_point)
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
            light_linestring=mock_linestring,
            road_id=0,
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mock_road_lanelet_mapping,
        )

    # For a straight line along x-axis, the signal at (10, 5, 2) should be:
    # - s ≈ 10.0 (x-coordinate along the spline)
    # - t ≈ 5.0 (y-offset from the spline, positive = left)
    assert abs(s - 10.0) < 1.0, f"Expected s ≈ 10.0, got {s}"
    assert abs(t - 5.0) < 1.0, f"Expected t ≈ 5.0, got {t}"


def test_calculate_signal_position_with_empty_linestring():
    """Test signal position calculation when linestring is empty."""
    mock_traffic_light = Mock()
    mock_traffic_light.stopLine = None

    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=0)
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        light_linestring=mock_linestring,
        road_id=0,
        lanelet_map=mock_lanelet_map,
        road_lanelet_mapping=mock_road_lanelet_mapping,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_without_spline():
    """Test signal position calculation when road has no lanelets."""
    mock_traffic_light = Mock()
    mock_traffic_light.stopLine = None
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0

    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=1)
    mock_linestring.__getitem__ = Mock(return_value=mock_point)
    mock_traffic_light.trafficLights = [mock_linestring]
    mock_traffic_light.id = 1

    # Mock empty lanelet list for road
    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = []

    # Should return default position (0.0, -4.0)
    s, t = SignalsAndControllers._calculate_signal_position(
        traffic_light=mock_traffic_light,
        light_linestring=mock_linestring,
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
    mock_traffic_light.stopLine = None
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = -3.0  # Negative y means right side of the road
    mock_point.z = 2.0

    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=1)
    mock_linestring.__getitem__ = Mock(return_value=mock_point)
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
            light_linestring=mock_linestring,
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
    mock_traffic_light.stopLine = None
    mock_point = Mock()
    mock_point.x = 10.0
    mock_point.y = 5.0
    mock_point.z = 2.0

    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=1)
    mock_linestring.__getitem__ = Mock(return_value=mock_point)
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
        light_linestring=mock_linestring,
        road_id=0,
        lanelet_map=mock_lanelet_map,
        road_lanelet_mapping=mock_road_lanelet_mapping,
    )

    assert s == 0.0
    assert t == -4.0


def test_calculate_signal_position_with_stop_line():
    """Test that stop line centroid is used when stop line is available."""
    mock_traffic_light = Mock()
    mock_traffic_light.id = 1

    # Stop line at x=15, y=0
    stop_pt1 = Mock(x=14.0, y=-2.0, z=0.0)
    stop_pt2 = Mock(x=16.0, y=2.0, z=0.0)
    mock_stop_line = Mock()
    mock_stop_line.__len__ = Mock(return_value=2)
    mock_stop_line.__getitem__ = Mock(side_effect=lambda i: [stop_pt1, stop_pt2][i])
    mock_traffic_light.stopLine = mock_stop_line

    # Light linestring at x=10 (should NOT be used)
    light_pt = Mock(x=10.0, y=5.0, z=8.0)
    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=1)
    mock_linestring.__getitem__ = Mock(return_value=light_pt)
    mock_traffic_light.trafficLights = [mock_linestring]

    # Straight road along x-axis
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

    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = [1]
    mock_lanelet_map.laneletLayer.get.return_value = Mock()

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_ref:
        mock_reference_line = Mock()
        mock_reference_line.centerline_2d = spline
        mock_ref.construct_from_lanelet_groups.return_value = mock_reference_line

        s, t = SignalsAndControllers._calculate_signal_position(
            traffic_light=mock_traffic_light,
            light_linestring=mock_linestring,
            road_id=0,
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mock_road_lanelet_mapping,
        )

    # Stop line centroid is at (15, 0), so s ≈ 15.0
    assert abs(s - 15.0) < 1.0, f"Expected s ≈ 15.0 (stop line), got {s}"


def test_calculate_signal_position_uses_linestring_centroid_fallback():
    """Test that linestring centroid (all points) is used when no stop line."""
    mock_traffic_light = Mock()
    mock_traffic_light.id = 1
    mock_traffic_light.stopLine = None

    # Two-point linestring: centroid x = (8+12)/2 = 10
    pt1 = Mock(x=8.0, y=4.0, z=5.0)
    pt2 = Mock(x=12.0, y=6.0, z=5.0)
    mock_linestring = Mock()
    mock_linestring.__len__ = Mock(return_value=2)
    mock_linestring.__getitem__ = Mock(side_effect=lambda i: [pt1, pt2][i])
    mock_traffic_light.trafficLights = [mock_linestring]

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

    mock_lanelet_map = Mock()
    mock_road_lanelet_mapping = Mock()
    mock_road_lanelet_mapping.get_lanelets_for_road.return_value = [1]
    mock_lanelet_map.laneletLayer.get.return_value = Mock()

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_ref:
        mock_reference_line = Mock()
        mock_reference_line.centerline_2d = spline
        mock_ref.construct_from_lanelet_groups.return_value = mock_reference_line

        s, t = SignalsAndControllers._calculate_signal_position(
            traffic_light=mock_traffic_light,
            light_linestring=mock_linestring,
            road_id=0,
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mock_road_lanelet_mapping,
        )

    # Centroid x = 10, so s ≈ 10.0
    assert abs(s - 10.0) < 1.0, f"Expected s ≈ 10.0 (centroid), got {s}"


def test_calculate_physical_position_centroid():
    """Test _calculate_physical_position returns centroid coordinates."""
    from autoware_lanelet2_to_opendrive.config import COORDINATE_OFFSET

    original_x = COORDINATE_OFFSET.x
    original_y = COORDINATE_OFFSET.y
    original_z = COORDINATE_OFFSET.z
    COORDINATE_OFFSET.x = 0.0
    COORDINATE_OFFSET.y = 0.0
    COORDINATE_OFFSET.z = 0.0

    try:
        pt1 = Mock(x=10.0, y=20.0, z=5.0)
        pt2 = Mock(x=14.0, y=24.0, z=7.0)
        pt3 = Mock(x=12.0, y=22.0, z=6.0)
        mock_ls = Mock()
        mock_ls.__len__ = Mock(return_value=3)
        mock_ls.__getitem__ = Mock(side_effect=lambda i: [pt1, pt2, pt3][i])

        pos = SignalsAndControllers._calculate_physical_position(
            light_linestring=mock_ls,
        )

        assert abs(pos.x - 12.0) < 1e-6
        assert abs(pos.y - 22.0) < 1e-6
        assert abs(pos.z - 6.0) < 1e-6
        # hdg: from pt1(10,20) to pt3(12,22) → atan2(2,2) = π/4
        expected_hdg = math.atan2(22.0 - 20.0, 12.0 - 10.0)
        assert abs(pos.hdg - expected_hdg) < 1e-6
    finally:
        COORDINATE_OFFSET.x = original_x
        COORDINATE_OFFSET.y = original_y
        COORDINATE_OFFSET.z = original_z


def test_calculate_physical_position_single_point():
    """Test _calculate_physical_position with single point has hdg=0."""
    from autoware_lanelet2_to_opendrive.config import COORDINATE_OFFSET

    original_x = COORDINATE_OFFSET.x
    original_y = COORDINATE_OFFSET.y
    original_z = COORDINATE_OFFSET.z
    COORDINATE_OFFSET.x = 0.0
    COORDINATE_OFFSET.y = 0.0
    COORDINATE_OFFSET.z = 0.0

    try:
        pt = Mock(x=5.0, y=10.0, z=3.0)
        mock_ls = Mock()
        mock_ls.__len__ = Mock(return_value=1)
        mock_ls.__getitem__ = Mock(return_value=pt)

        pos = SignalsAndControllers._calculate_physical_position(
            light_linestring=mock_ls,
        )

        assert abs(pos.x - 5.0) < 1e-6
        assert abs(pos.y - 10.0) < 1e-6
        assert abs(pos.z - 3.0) < 1e-6
        assert pos.hdg == 0.0  # Single point → no direction
    finally:
        COORDINATE_OFFSET.x = original_x
        COORDINATE_OFFSET.y = original_y
        COORDINATE_OFFSET.z = original_z
