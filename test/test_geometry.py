"""Tests for geometry utility functions."""

import numpy as np
from autoware_lanelet2_to_opendrive.geometry import (
    point_to_line_segment_distance,
    ArcLengthParameterizedCatmullRomSpline,
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


class TestArcLengthParameterizedCatmullRomSpline:
    """Test cases for ArcLengthParameterizedCatmullRomSpline function."""

    def test_create_arc_length_spline(self):
        """Test creating arc length parameterized spline."""
        # Create a simple curved path
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
            ]
        )

        # Create arc length parameterized spline
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # Test that it returns an ArcLengthParameterizedCatmullRomSpline
        assert isinstance(spline, ArcLengthParameterizedCatmullRomSpline)

        # Test that total_length is positive
        assert spline.total_length > 0

        # Test that evaluation at start and end works
        start_point = spline.evaluate(0.0)
        end_point = spline.evaluate(spline.total_length)

        assert start_point.shape[0] == 3
        assert end_point.shape[0] == 3

    def test_arc_length_spline_with_insufficient_points(self):
        """Test that function raises error with insufficient points."""
        # Only 3 points (need at least 4 for Catmull-Rom)
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 1.0, 0.0],
            ]
        )

        # Should raise ValueError
        try:
            ArcLengthParameterizedCatmullRomSpline(points)
            assert False, "Should have raised ValueError"
        except ValueError as e:
            assert "at least 4 points" in str(e)

    def test_arc_length_consistency(self):
        """Test that arc length parameterization is consistent."""
        # Create a straight line (should have predictable arc length)
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ]
        )

        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # For a straight line, the arc length should be approximately
        # the distance between start and end points
        total_length = spline.total_length
        assert (
            total_length > 2.5
        )  # Should be close to 3 but may vary due to spline interpolation

        # Test evaluation at quarter, half, and three-quarter points
        quarter_point = spline.evaluate(total_length * 0.25)
        half_point = spline.evaluate(total_length * 0.5)
        three_quarter_point = spline.evaluate(total_length * 0.75)

        # For a straight line along x-axis, y should remain close to 0
        assert abs(quarter_point[1]) < 0.1
        assert abs(half_point[1]) < 0.1
        assert abs(three_quarter_point[1]) < 0.1

    def test_custom_alpha_parameter(self):
        """Test that custom alpha parameter is respected."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
            ]
        )

        # Test different alpha values
        spline_uniform = ArcLengthParameterizedCatmullRomSpline(points, alpha=0.0)
        spline_centripetal = ArcLengthParameterizedCatmullRomSpline(points, alpha=0.5)
        spline_chordal = ArcLengthParameterizedCatmullRomSpline(points, alpha=1.0)

        # All should be valid ArcLengthParameterizedCatmullRomSpline objects
        assert isinstance(spline_uniform, ArcLengthParameterizedCatmullRomSpline)
        assert isinstance(spline_centripetal, ArcLengthParameterizedCatmullRomSpline)
        assert isinstance(spline_chordal, ArcLengthParameterizedCatmullRomSpline)

        # They should have different total lengths due to different spline types
        length_uniform = spline_uniform.total_length
        length_centripetal = spline_centripetal.total_length
        length_chordal = spline_chordal.total_length

        # Lengths should be positive and may differ
        assert length_uniform > 0
        assert length_centripetal > 0
        assert length_chordal > 0

    def test_frenet_coordinate_calculation(self):
        """Test Frenet coordinate calculation functionality."""
        # Create a curved path for testing Frenet coordinates
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
            ]
        )

        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # Test Frenet coordinate evaluation at midpoint
        mid_s = spline.total_length / 2
        frenet_frame = spline.evaluate(mid_s, frenet=True)

        # Check that all expected keys are present
        assert "position" in frenet_frame
        assert "tangent" in frenet_frame
        assert "normal" in frenet_frame

        position = frenet_frame["position"]
        tangent = frenet_frame["tangent"]
        normal = frenet_frame["normal"]

        # Check dimensions
        assert position.shape[0] == 3
        assert tangent.shape[0] == 3
        assert normal.shape[0] == 3

        # Check that tangent and normal are unit vectors
        assert abs(np.linalg.norm(tangent) - 1.0) < 1e-10
        assert abs(np.linalg.norm(normal) - 1.0) < 1e-10

        # Check that tangent and normal are perpendicular (in XY plane)
        dot_product = np.dot(tangent[:2], normal[:2])
        assert abs(dot_product) < 1e-10

        # Normal should be in XY plane (z=0)
        assert abs(normal[2]) < 1e-10

    def test_frenet_vs_regular_evaluation(self):
        """Test that frenet=False gives same position as regular evaluation."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
            ]
        )

        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # Test at several points
        test_s_values = [
            0.0,
            spline.total_length * 0.25,
            spline.total_length * 0.5,
            spline.total_length * 0.75,
            spline.total_length,
        ]

        for s in test_s_values:
            if s > spline.total_length:
                s = spline.total_length

            regular_pos = spline.evaluate(s, frenet=False)
            frenet_frame = spline.evaluate(s, frenet=True)
            frenet_pos = frenet_frame["position"]

            # Positions should be identical
            assert np.allclose(regular_pos, frenet_pos, atol=1e-12)

    def test_edge_cases_frenet(self):
        """Test Frenet coordinate calculation at edge cases."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ]
        )

        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # Test at start
        frenet_start = spline.evaluate(0.0, frenet=True)
        assert "position" in frenet_start
        assert "tangent" in frenet_start
        assert "normal" in frenet_start

        # Test at end
        frenet_end = spline.evaluate(spline.total_length, frenet=True)
        assert "position" in frenet_end
        assert "tangent" in frenet_end
        assert "normal" in frenet_end

        # For a straight line along x-axis, tangent should be approximately [1,0,0]
        # and normal should be approximately [0,1,0]
        tangent_start = frenet_start["tangent"]
        normal_start = frenet_start["normal"]

        assert (
            abs(tangent_start[0] - 1.0) < 0.1
        )  # Allow some tolerance for spline approximation
        assert abs(tangent_start[1]) < 0.1
        assert abs(normal_start[0]) < 0.1
        assert abs(abs(normal_start[1]) - 1.0) < 0.1
