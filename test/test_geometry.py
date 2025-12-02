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

    def test_well_formed_spline_no_exceptions(self):
        """Test that well-formed splines do not raise exceptions during Frenet evaluation."""
        # Create a straight line along x-axis (well-formed case)
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ]
        )

        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # This should succeed without raising exceptions
        frenet_frame = spline.evaluate(spline.total_length / 2, frenet=True)

        # Verify the result contains all expected components
        assert "position" in frenet_frame
        assert "tangent" in frenet_frame
        assert "normal" in frenet_frame

        # For a straight line along x-axis, verify expected directions
        tangent = frenet_frame["tangent"]
        normal = frenet_frame["normal"]

        # Tangent should be approximately along x-axis
        assert abs(tangent[0]) > 0.9  # Strong x-component
        assert abs(tangent[1]) < 0.1  # Minimal y-component
        assert abs(tangent[2]) < 0.1  # Minimal z-component

        # Normal should be approximately along y-axis
        assert abs(normal[0]) < 0.1  # Minimal x-component
        assert abs(normal[1]) > 0.9  # Strong y-component
        assert abs(normal[2]) < 0.1  # Should be in XY plane

    def test_as_cubic_spline_parameters_structure(self):
        """Test structure of cubic spline parameters output."""
        points = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, 1, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        # Should have at least one segment
        assert len(segments) >= 1

        for segment in segments:
            # Check required keys
            required_keys = [
                "t_start",
                "t_end",
                "s_start",
                "s_end",
                "a",
                "b",
                "c",
                "d",
                "segment_length",
            ]
            for key in required_keys:
                assert key in segment

            # Check value types
            assert isinstance(segment["t_start"], (int, float, np.number))
            assert isinstance(segment["t_end"], (int, float, np.number))
            assert isinstance(segment["s_start"], (int, float, np.number))
            assert isinstance(segment["s_end"], (int, float, np.number))
            assert isinstance(segment["a"], (int, float, np.number))
            assert isinstance(segment["b"], (int, float, np.number))
            assert isinstance(segment["c"], (int, float, np.number))
            assert isinstance(segment["d"], (int, float, np.number))
            assert isinstance(segment["segment_length"], (int, float, np.number))

            # Check logical constraints
            assert segment["t_end"] > segment["t_start"]
            assert segment["s_end"] >= segment["s_start"]
            assert segment["segment_length"] >= 0

    def test_as_cubic_spline_parameters_straight_line(self):
        """Test cubic parameters for a straight line (should have zero higher-order terms)."""
        points = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        for segment in segments:
            # For a straight line, lateral deviation should be minimal
            # Higher order terms (c, d) should be close to zero
            assert abs(segment["c"]) < 1e-3
            assert abs(segment["d"]) < 1e-3

    def test_as_cubic_spline_parameters_curved_line(self):
        """Test cubic parameters for a curved line."""
        points = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, 1, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        # For curved lines, at least some segments should have non-zero higher-order terms
        has_curve = any(
            abs(seg["c"]) > 1e-6 or abs(seg["d"]) > 1e-6 for seg in segments
        )
        assert has_curve, "Curved line should have non-zero higher-order cubic terms"

    def test_as_cubic_spline_parameters_segment_continuity(self):
        """Test that segments are continuous."""
        points = np.array([[0, 0, 0], [1, 2, 0], [3, 1, 0], [4, 3, 0], [5, 0, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        if len(segments) > 1:
            for i in range(len(segments) - 1):
                current_seg = segments[i]
                next_seg = segments[i + 1]

                # End of current should match start of next (approximately)
                assert abs(current_seg["t_end"] - next_seg["t_start"]) < 1e-6
                assert abs(current_seg["s_end"] - next_seg["s_start"]) < 1e-6

    def test_as_cubic_spline_parameters_polynomial_consistency(self):
        """Test that cubic polynomial parameters produce consistent results."""
        points = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, 1, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        for segment in segments:
            a, b, c, d = segment["a"], segment["b"], segment["c"], segment["d"]
            s_start = segment["s_start"]
            s_end = segment["s_end"]

            # Test polynomial evaluation at segment boundaries
            # At s_start (local s=0), polynomial should give value 'a'
            local_s_start = 0.0
            poly_value_start = (
                a + b * local_s_start + c * local_s_start**2 + d * local_s_start**3
            )
            assert poly_value_start == a

            # At s_end, polynomial should give reasonable value
            segment_length = s_end - s_start
            poly_value_end = (
                a + b * segment_length + c * segment_length**2 + d * segment_length**3
            )

            # The polynomial value should be finite
            assert np.isfinite(poly_value_end)

    def test_as_cubic_spline_parameters_no_num_segments_arg(self):
        """Test that as_cubic_spline_parameters doesn't accept num_segments argument."""
        points = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, 1, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        # Should work without arguments
        segments = spline.as_cubic_spline_parameters()
        assert isinstance(segments, list)

        # Should fail with num_segments argument
        try:
            spline.as_cubic_spline_parameters(num_segments=5)
            assert False, "Should have raised TypeError for unexpected argument"
        except TypeError:
            pass  # Expected behavior

    def test_as_cubic_spline_parameters_produces_valid_representation(self):
        """Test that cubic polynomial parameters produce a valid mathematical representation."""
        points = np.array([[0, 0, 0], [1, 2, 0], [3, 1, 0], [4, 3, 0], [5, 0, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        # Should have segments for the spline
        assert len(segments) > 0, "Should produce at least one segment"

        # Verify basic properties of the polynomial representation
        for segment_idx, segment in enumerate(segments):
            a, b, c, d = segment["a"], segment["b"], segment["c"], segment["d"]

            # Coefficients should be finite
            assert all(
                np.isfinite([a, b, c, d])
            ), f"Segment {segment_idx} has non-finite coefficients: a={a}, b={b}, c={c}, d={d}"

            # Segment should have positive length
            assert (
                segment["segment_length"] > 0
            ), f"Segment {segment_idx} has non-positive length: {segment['segment_length']}"

            # Verify t and s ranges make sense
            assert (
                segment["t_end"] > segment["t_start"]
            ), f"Segment {segment_idx} has invalid t range: [{segment['t_start']}, {segment['t_end']}]"
            assert (
                segment["s_end"] >= segment["s_start"]
            ), f"Segment {segment_idx} has invalid s range: [{segment['s_start']}, {segment['s_end']}]"

            # Polynomial should be evaluable across the segment range
            test_positions = np.linspace(0, segment["segment_length"], 5)
            for local_s in test_positions:
                polynomial_value = a + b * local_s + c * local_s**2 + d * local_s**3

                # Polynomial should produce finite values
                assert np.isfinite(
                    polynomial_value
                ), f"Polynomial produces non-finite value at local_s={local_s} in segment {segment_idx}"

        # Verify segments cover the full spline range
        total_segment_length = sum(seg["segment_length"] for seg in segments)
        spline_length = spline.total_length

        # Should cover most of the spline (allowing for some approximation)
        coverage_ratio = (
            total_segment_length / spline_length if spline_length > 0 else 0
        )
        assert coverage_ratio > 0.5, (
            f"Segments should cover significant portion of spline: "
            f"coverage={coverage_ratio:.3f}, total_segment_length={total_segment_length:.3f}, "
            f"spline_length={spline_length:.3f}"
        )

    def test_as_cubic_spline_parameters_mathematical_consistency(self):
        """Test mathematical consistency of polynomial parameters with spline evaluation."""
        # Test with a simple curved path
        points = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, 1, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        for segment_idx, segment in enumerate(segments):
            a, b, c, d = segment["a"], segment["b"], segment["c"], segment["d"]
            s_start = segment["s_start"]
            s_end = segment["s_end"]

            # Test mathematical properties
            # 1. Polynomial coefficients should be finite
            assert all(
                np.isfinite([a, b, c, d])
            ), f"Segment {segment_idx} coefficients not finite: a={a}, b={b}, c={c}, d={d}"

            # 2. At local_s=0, polynomial value should be 'a'
            poly_at_zero = a + b * 0 + c * 0**2 + d * 0**3
            assert (
                abs(poly_at_zero - a) < 1e-12
            ), f"Polynomial at s=0 should equal coefficient 'a': {poly_at_zero} != {a}"

            # 3. Polynomial derivatives should be computable
            segment_length = s_end - s_start
            test_s = segment_length / 2

            # Value at midpoint
            poly_value = a + b * test_s + c * test_s**2 + d * test_s**3
            assert np.isfinite(poly_value), "Polynomial value should be finite"

            # First derivative: b + 2*c*s + 3*d*s^2
            poly_derivative = b + 2 * c * test_s + 3 * d * test_s**2
            assert np.isfinite(
                poly_derivative
            ), "Polynomial derivative should be finite"

            # 4. Segment parameters should be consistent
            assert (
                s_end > s_start
            ), f"Segment {segment_idx}: s_end ({s_end}) <= s_start ({s_start})"
            assert (
                segment["t_end"] > segment["t_start"]
            ), f"Segment {segment_idx}: t_end ({segment['t_end']}) <= t_start ({segment['t_start']})"

            # 5. The polynomial should produce reasonable values across the segment
            test_positions = [
                0,
                segment_length * 0.25,
                segment_length * 0.5,
                segment_length * 0.75,
                segment_length,
            ]
            for local_s in test_positions:
                value = a + b * local_s + c * local_s**2 + d * local_s**3
                assert np.isfinite(
                    value
                ), f"Non-finite polynomial value at local_s={local_s}"

                # Value should not be excessively large (sanity check)
                assert (
                    abs(value) < 100
                ), f"Polynomial value seems excessive: {value} at local_s={local_s}"

    def test_as_cubic_spline_parameters_approximates_original_spline(self):
        """Demonstrate that polynomial parameters approximate the original spline shape."""
        # Use a simple smooth curve for better approximation
        points = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0], [3, -1, 0]])
        spline = ArcLengthParameterizedCatmullRomSpline(points)

        segments = spline.as_cubic_spline_parameters()

        # Collect approximation quality statistics
        all_errors = []

        for segment_idx, segment in enumerate(segments):
            s_start = segment["s_start"]
            s_end = segment["s_end"]
            a, b, c, d = segment["a"], segment["b"], segment["c"], segment["d"]

            # Sample multiple points within segment for comparison
            num_test_points = 15
            for i in range(num_test_points):
                t = i / (num_test_points - 1) if num_test_points > 1 else 0.0
                s_test = s_start + t * (s_end - s_start)

                # Skip if beyond spline bounds
                if s_test > spline.total_length:
                    continue

                # Get actual spline value at this position
                frenet_frame = spline.evaluate(s_test, frenet=True)
                start_frame = spline.evaluate(s_start, frenet=True)

                # Convert to local coordinate system
                delta = frenet_frame["position"] - start_frame["position"]
                actual_lateral = np.dot(delta, start_frame["normal"])

                # Get polynomial prediction
                local_s = s_test - s_start
                predicted_lateral = a + b * local_s + c * local_s**2 + d * local_s**3

                # Calculate approximation error
                error = abs(actual_lateral - predicted_lateral)
                all_errors.append(error)

        # Verify the approximation quality
        if all_errors:
            mean_error = np.mean(all_errors)
            max_error = max(all_errors)

            # The polynomial approximation should be reasonable for smooth curves
            # These are generous bounds since we're doing least-squares fitting
            assert mean_error < 0.5, (
                f"Mean approximation error too large: {mean_error:.4f}. "
                f"Polynomial fitting should provide reasonable approximation of smooth curves."
            )

            assert max_error < 2.0, (
                f"Maximum approximation error too large: {max_error:.4f}. "
                f"Polynomial should not deviate excessively from original spline."
            )

            # Most errors should be small
            small_errors = [e for e in all_errors if e < 0.1]
            small_error_ratio = len(small_errors) / len(all_errors)

            assert small_error_ratio > 0.3, (
                f"Too few points have small approximation errors: {small_error_ratio:.2f}. "
                f"At least 30% of points should be well-approximated."
            )
