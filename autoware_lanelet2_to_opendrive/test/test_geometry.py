"""Tests for geometry utility functions."""

import lanelet2
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.geometry import (
    compute_point_layer_bounds,
    point_to_line_segment_distance,
)


class TestPointToLineSegmentDistance:
    """Test cases for point_to_line_segment_distance function."""

    def test_distance_to_horizontal_line_segment(self):
        """Test distance calculation to a horizontal line segment."""
        # Horizontal line from (0,0) to (10,0)
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([10.0, 0.0, 0.0])

        # Point above the line
        point = np.array([5.0, 3.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Distance should be 3.0 (perpendicular distance)
        assert abs(distance - 3.0) < 1e-10

    def test_distance_to_vertical_line_segment(self):
        """Test distance calculation to a vertical line segment."""
        # Vertical line from (0,0) to (0,10)
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([0.0, 10.0, 0.0])

        # Point to the right of the line
        point = np.array([4.0, 5.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Distance should be 4.0 (perpendicular distance)
        assert abs(distance - 4.0) < 1e-10

    def test_distance_to_diagonal_line_segment(self):
        """Test distance calculation to a diagonal line segment."""
        # Diagonal line from (0,0) to (3,4) - length 5
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([3.0, 4.0, 0.0])

        # Point at (4,3) - should be distance 5 from the line y = (4/3)x
        point = np.array([4.0, 3.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Expected distance from point (4,3) to line 4x - 3y = 0 is |4*4 - 3*3|/5 = 7/5 = 1.4
        expected = 1.4
        assert abs(distance - expected) < 1e-10

    def test_distance_to_point_on_line(self):
        """Test distance when point is exactly on the line segment."""
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([10.0, 0.0, 0.0])

        # Point on the line segment
        point = np.array([5.0, 0.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Distance should be 0
        assert abs(distance) < 1e-10

    def test_distance_to_line_segment_endpoints(self):
        """Test distance calculation when closest point is an endpoint."""
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([5.0, 0.0, 0.0])

        # Point beyond the start
        point1 = np.array([-3.0, 4.0, 0.0])
        distance1 = point_to_line_segment_distance(point1, seg_start, seg_end)
        expected1 = np.sqrt(9 + 16)  # Distance to (0,0)
        assert abs(distance1 - expected1) < 1e-10

        # Point beyond the end
        point2 = np.array([8.0, 4.0, 0.0])
        distance2 = point_to_line_segment_distance(point2, seg_start, seg_end)
        expected2 = np.sqrt(9 + 16)  # Distance to (5,0)
        assert abs(distance2 - expected2) < 1e-10

    def test_zero_length_segment(self):
        """Test behavior with zero-length line segment."""
        seg_start = np.array([2.0, 3.0, 0.0])
        seg_end = np.array([2.0, 3.0, 0.0])  # Same point

        point = np.array([5.0, 7.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Should return distance to the point
        expected = np.sqrt((5 - 2) ** 2 + (7 - 3) ** 2)
        assert abs(distance - expected) < 1e-10

    def test_directional_distance_perpendicular(self):
        """Test directional distance calculation with perpendicular direction."""
        # Horizontal line from (0,0) to (10,0)
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([10.0, 0.0, 0.0])

        # Point above the line
        point = np.array([5.0, 3.0, 0.0])

        # Direction pointing down (toward the line)
        direction = np.array([0.0, -1.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end, direction)

        # Should return 3.0 (perpendicular distance)
        assert abs(distance - 3.0) < 1e-10

    def test_directional_distance_parallel(self):
        """Test directional distance calculation with parallel direction."""
        # Horizontal line from (0,0) to (10,0)
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([10.0, 0.0, 0.0])

        # Point above the line
        point = np.array([5.0, 3.0, 0.0])

        # Direction parallel to the line
        direction = np.array([1.0, 0.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end, direction)

        # Should return None (no intersection)
        assert distance is None

    def test_directional_distance_wrong_direction(self):
        """Test directional distance calculation with wrong direction."""
        # Horizontal line from (0,0) to (10,0)
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([10.0, 0.0, 0.0])

        # Point above the line
        point = np.array([5.0, 3.0, 0.0])

        # Direction pointing up (away from the line)
        direction = np.array([0.0, 1.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end, direction)

        # Should return None (wrong direction)
        assert distance is None

    def test_directional_distance_outside_segment(self):
        """Test directional distance when intersection is outside segment."""
        # Short horizontal line from (0,0) to (2,0)
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([2.0, 0.0, 0.0])

        # Point far to the right and above
        point = np.array([10.0, 3.0, 0.0])

        # Direction pointing down
        direction = np.array([0.0, -1.0, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end, direction)

        # Should return None (intersection outside segment)
        assert distance is None

    def test_3d_coordinates(self):
        """Test that function works correctly with 3D coordinates (ignoring z)."""
        # Line segment with different z coordinates
        seg_start = np.array([0.0, 0.0, 5.0])
        seg_end = np.array([10.0, 0.0, 10.0])

        # Point with different z coordinate
        point = np.array([5.0, 3.0, 15.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Should ignore z coordinates, distance should be 3.0
        assert abs(distance - 3.0) < 1e-10

    def test_numerical_precision(self):
        """Test numerical precision with very small distances."""
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([1.0, 0.0, 0.0])

        # Point very close to the line
        point = np.array([0.5, 1e-12, 0.0])

        distance = point_to_line_segment_distance(point, seg_start, seg_end)

        # Should return the small distance
        assert abs(distance - 1e-12) < 1e-15


class TestComputePointLayerBounds:
    """Tests for compute_point_layer_bounds."""

    def test_synthetic_minimal_map_returns_exact_bounds(self):
        """Hand-placed Point3d instances yield exact min/max."""
        lanelet_map = lanelet2.core.LaneletMap()
        # Build a minimal LineString3d so the points end up in pointLayer.
        points = [
            lanelet2.core.Point3d(lanelet2.core.getId(), -3.0, 7.0, 0.0),
            lanelet2.core.Point3d(lanelet2.core.getId(), 11.0, -2.5, 0.0),
            lanelet2.core.Point3d(lanelet2.core.getId(), 4.0, 1.0, 0.0),
        ]
        ls = lanelet2.core.LineString3d(lanelet2.core.getId(), points)
        lanelet_map.add(ls)

        min_x, min_y, max_x, max_y = compute_point_layer_bounds(lanelet_map)

        assert min_x == pytest.approx(-3.0)
        assert min_y == pytest.approx(-2.5)
        assert max_x == pytest.approx(11.0)
        assert max_y == pytest.approx(7.0)

    def test_real_fixture_bounds_are_finite_and_nondegenerate(self, lanelet_map):
        """The session-scoped nishishinjuku fixture yields a non-degenerate box."""
        import math

        min_x, min_y, max_x, max_y = compute_point_layer_bounds(lanelet_map)

        for v in (min_x, min_y, max_x, max_y):
            assert math.isfinite(v)
        assert min_x < max_x
        assert min_y < max_y

    def test_empty_point_layer_raises(self):
        """An empty LaneletMap must raise ValueError, not silently return zeros."""
        empty = lanelet2.core.LaneletMap()
        with pytest.raises(ValueError):
            compute_point_layer_bounds(empty)
