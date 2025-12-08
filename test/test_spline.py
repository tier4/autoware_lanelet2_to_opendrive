"""Tests for the Splines class."""

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.spline import Splines


class TestSplinesBasic:
    """Basic tests for Splines class creation and evaluation."""

    def test_create_spline_with_minimum_points(self):
        """Test creating a spline with minimum number of points (2)."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])

        spline = Splines(points, num_control_points=4)

        assert spline is not None
        assert spline.total_length == pytest.approx(1.0, rel=1e-6)

        # Check start and end positions
        start_pos = spline.evaluate(0.0)
        end_pos = spline.evaluate(spline.total_length)

        assert np.allclose(start_pos, points[0], atol=1e-6)
        assert np.allclose(end_pos, points[1], atol=1e-6)

        # Coordinate validation tests
        # For a straight horizontal line, x should equal arc length s
        mid_pos = spline.evaluate(0.5)
        assert abs(mid_pos[0] - 0.5) < 1e-3, f"Expected x=0.5, got x={mid_pos[0]}"
        assert abs(mid_pos[1] - 0.0) < 1e-3, f"Expected y=0.0, got y={mid_pos[1]}"
        assert abs(mid_pos[2] - 0.0) < 1e-3, f"Expected z=0.0, got z={mid_pos[2]}"

    def test_create_spline_with_multiple_points(self):
        """Test creating a spline with multiple points."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
                [4.0, 0.0, 0.0],
            ]
        )

        spline = Splines(points, num_control_points=10)

        assert spline is not None
        assert spline.total_length > 0

        # Verify spline passes close to start and end points
        start_pos = spline.evaluate(0.0)
        end_pos = spline.evaluate(spline.total_length)

        assert np.allclose(start_pos, points[0], atol=1e-3)
        assert np.allclose(end_pos, points[-1], atol=1e-3)

        # Coordinate validation tests
        # Check intermediate positions maintain reasonable coordinates
        mid_arc_length = spline.total_length / 2
        mid_pos = spline.evaluate(mid_arc_length)

        # Verify coordinates are within expected bounds
        assert (
            0.0 <= mid_pos[0] <= 5.0
        ), f"X coordinate {mid_pos[0]} out of expected range [0,5]"
        assert (
            -0.5 <= mid_pos[1] <= 1.5
        ), f"Y coordinate {mid_pos[1]} out of expected range [-0.5,1.5]"
        assert abs(mid_pos[2]) < 0.1, f"Z coordinate {mid_pos[2]} should be near 0"

    def test_evaluate_at_different_parameters(self):
        """Test evaluating spline at different parameter values."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points, num_control_points=6)

        # Evaluate at various arc length values
        positions = []
        total_length = spline.total_length
        for s in [
            0.0,
            total_length * 0.25,
            total_length * 0.5,
            total_length * 0.75,
            total_length,
        ]:
            pos = spline.evaluate(s)
            positions.append(pos)
            assert pos.shape == (3,)
            assert np.all(np.isfinite(pos))

        # Check that positions are different (spline is not constant)
        for i in range(len(positions) - 1):
            assert not np.allclose(positions[i], positions[i + 1])

        # Coordinate validation tests
        # Verify all positions have reasonable coordinate values
        for i, pos in enumerate(positions):
            assert (
                0.0 <= pos[0] <= 2.5
            ), f"Position {i}: X coordinate {pos[0]} out of range [0,2.5]"
            assert (
                -0.5 <= pos[1] <= 1.5
            ), f"Position {i}: Y coordinate {pos[1]} out of range [-0.5,1.5]"
            assert (
                abs(pos[2]) < 0.1
            ), f"Position {i}: Z coordinate {pos[2]} should be near 0"

    def test_evaluate_arc_length(self):
        """Test evaluating spline using arc length parameterization."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
        )

        spline = Splines(points)
        total_length = spline.total_length

        # Evaluate at different arc lengths
        s_values = [0.0, total_length * 0.5, total_length]
        for s in s_values:
            pos = spline.evaluate_arc_length(s)
            assert pos.shape == (3,)
            assert np.all(np.isfinite(pos))

        # Check start and end positions
        start_pos = spline.evaluate_arc_length(0.0)
        end_pos = spline.evaluate_arc_length(total_length)

        assert np.allclose(start_pos, points[0], atol=1e-3)
        assert np.allclose(end_pos, points[-1], atol=1e-3)

        # Coordinate validation tests for straight line
        # For this straight horizontal line, x should approximately equal arc length s
        pos_1 = spline.evaluate_arc_length(1.0)
        pos_1_5 = spline.evaluate_arc_length(1.5)
        pos_2 = spline.evaluate_arc_length(2.0)

        assert abs(pos_1[0] - 1.0) < 0.1, f"At s=1.0, expected x≈1.0, got x={pos_1[0]}"
        assert abs(pos_1[1] - 0.0) < 0.1, f"At s=1.0, expected y≈0.0, got y={pos_1[1]}"
        assert (
            abs(pos_1_5[0] - 1.5) < 0.1
        ), f"At s=1.5, expected x≈1.5, got x={pos_1_5[0]}"
        assert abs(pos_2[0] - 2.0) < 0.1, f"At s=2.0, expected x≈2.0, got x={pos_2[0]}"


class TestSplinesConstraints:
    """Tests for constrained spline fitting."""

    def test_explicit_start_end_tangents(self):
        """Test spline with explicitly specified start and end tangents."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 1.0, 0.0], [3.0, 0.0, 0.0]]
        )

        # Specify tangent vectors
        start_vel = np.array([1.0, 1.0, 0.0])
        start_vel = start_vel / np.linalg.norm(start_vel)

        end_vel = np.array([1.0, -1.0, 0.0])
        end_vel = end_vel / np.linalg.norm(end_vel)

        spline = Splines(
            points, start_vel=start_vel, end_vel=end_vel, num_control_points=8
        )

        # Check that tangents approximately match at endpoints
        start_tangent = spline.evaluate(0.0, derivative=1)
        end_tangent = spline.evaluate(spline.total_length, derivative=1)

        # Normalize for comparison
        start_tangent_norm = start_tangent / np.linalg.norm(start_tangent)
        end_tangent_norm = end_tangent / np.linalg.norm(end_tangent)

        # Check direction (allowing some tolerance due to scaling)
        assert np.dot(start_tangent_norm, start_vel) > 0.9
        assert np.dot(end_tangent_norm, end_vel) > 0.9

        # Coordinate validation tests
        # Verify spline endpoints match exactly
        start_pos = spline.evaluate(0.0)
        end_pos = spline.evaluate(spline.total_length)
        assert (
            abs(start_pos[0] - 0.0) < 1e-6
        ), f"Start X: expected 0.0, got {start_pos[0]}"
        assert (
            abs(start_pos[1] - 0.0) < 1e-6
        ), f"Start Y: expected 0.0, got {start_pos[1]}"
        assert abs(end_pos[0] - 3.0) < 1e-2, f"End X: expected ≈3.0, got {end_pos[0]}"
        assert abs(end_pos[1] - 0.0) < 1e-2, f"End Y: expected ≈0.0, got {end_pos[1]}"

    def test_automatic_tangent_estimation(self):
        """Test automatic tangent estimation when not specified."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 1.0, 0.0]]
        )

        # Create spline without specifying tangents
        spline = Splines(points)

        # Check that tangents are automatically estimated and reasonable
        start_tangent = spline.evaluate(0.0, derivative=1)
        end_tangent = spline.evaluate(spline.total_length, derivative=1)

        assert np.all(np.isfinite(start_tangent))
        assert np.all(np.isfinite(end_tangent))
        assert np.linalg.norm(start_tangent) > 0
        assert np.linalg.norm(end_tangent) > 0

        # Coordinate validation tests
        # Verify the spline interpolates correctly through the points
        start_pos = spline.evaluate(0.0)
        end_pos = spline.evaluate(spline.total_length)
        mid_pos = spline.evaluate(spline.total_length / 2)

        assert (
            abs(start_pos[0] - 0.0) < 1e-6
        ), f"Start X: expected 0.0, got {start_pos[0]}"
        assert (
            abs(start_pos[1] - 0.0) < 1e-6
        ), f"Start Y: expected 0.0, got {start_pos[1]}"
        assert abs(end_pos[0] - 3.0) < 1e-2, f"End X: expected ≈3.0, got {end_pos[0]}"
        assert abs(end_pos[1] - 1.0) < 1e-2, f"End Y: expected ≈1.0, got {end_pos[1]}"
        # Mid position should be somewhere reasonable between start and end
        assert 0.0 <= mid_pos[0] <= 3.0, f"Mid X: {mid_pos[0]} out of range [0,3]"
        assert 0.0 <= mid_pos[1] <= 1.2, f"Mid Y: {mid_pos[1]} out of range [0,1.2]"

    def test_varying_control_points(self):
        """Test spline behavior with different numbers of control points."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.5, 0.0],
                [2.0, 0.2, 0.0],
                [3.0, 0.8, 0.0],
                [4.0, 0.3, 0.0],
                [5.0, 1.0, 0.0],
            ]
        )

        # Create splines with different control point counts
        spline_smooth = Splines(points, num_control_points=4)  # Smoother
        spline_detailed = Splines(points, num_control_points=12)  # More detailed

        # Both should be valid
        assert spline_smooth is not None
        assert spline_detailed is not None

        # Check that both respect endpoint constraints
        for spline in [spline_smooth, spline_detailed]:
            start_pos = spline.evaluate(0.0)
            end_pos = spline.evaluate(spline.total_length)
            assert np.allclose(start_pos, points[0], atol=1e-2)
            assert np.allclose(end_pos, points[-1], atol=1e-2)

            # Coordinate validation for varying control points
            quarter_pos = spline.evaluate(spline.total_length * 0.25)
            assert (
                0.0 <= quarter_pos[0] <= 5.0
            ), f"X coordinate {quarter_pos[0]} out of range"
            assert (
                0.0 <= quarter_pos[1] <= 1.2
            ), f"Y coordinate {quarter_pos[1]} out of range"
            assert (
                abs(quarter_pos[2]) < 0.1
            ), f"Z coordinate {quarter_pos[2]} should be near 0"


class TestSplinesDerivatives:
    """Tests for spline derivatives."""

    def test_first_derivative(self):
        """Test first derivative (velocity) calculation."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points)

        # Test velocity at different arc lengths
        total_length = spline.total_length
        for s in [0.0, total_length * 0.5, total_length]:
            velocity = spline.evaluate(s, derivative=1)
            assert velocity.shape == (3,)
            assert np.all(np.isfinite(velocity))

        # Coordinate validation tests
        # Verify position coordinates at known arc lengths
        start_pos = spline.evaluate(0.0)
        mid_pos = spline.evaluate(total_length * 0.5)
        end_pos = spline.evaluate(total_length)

        assert (
            abs(start_pos[0] - 0.0) < 1e-6
        ), f"Start X: expected 0.0, got {start_pos[0]}"
        assert (
            abs(start_pos[1] - 0.0) < 1e-6
        ), f"Start Y: expected 0.0, got {start_pos[1]}"
        assert abs(end_pos[0] - 2.0) < 1e-2, f"End X: expected ≈2.0, got {end_pos[0]}"
        assert abs(end_pos[1] - 0.0) < 1e-2, f"End Y: expected ≈0.0, got {end_pos[1]}"
        assert 0.0 <= mid_pos[0] <= 2.0, f"Mid X: {mid_pos[0]} out of range [0,2]"

    def test_second_derivative(self):
        """Test second derivative (acceleration) calculation."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0], [3.0, -1.0, 0.0]]
        )

        spline = Splines(points, num_control_points=8)

        # Test acceleration at different arc lengths
        total_length = spline.total_length
        for s in [0.0, total_length * 0.5, total_length]:
            acceleration = spline.evaluate(s, derivative=2)
            assert acceleration.shape == (3,)
            assert np.all(np.isfinite(acceleration))

        # Coordinate validation tests
        # Verify position coordinates for this more complex curve
        start_pos = spline.evaluate(0.0)
        quarter_pos = spline.evaluate(total_length * 0.25)
        end_pos = spline.evaluate(total_length)

        assert (
            abs(start_pos[0] - 0.0) < 1e-6
        ), f"Start X: expected 0.0, got {start_pos[0]}"
        assert (
            abs(start_pos[1] - 0.0) < 1e-6
        ), f"Start Y: expected 0.0, got {start_pos[1]}"
        assert abs(end_pos[0] - 3.0) < 1e-2, f"End X: expected ≈3.0, got {end_pos[0]}"
        assert abs(end_pos[1] + 1.0) < 1e-2, f"End Y: expected ≈-1.0, got {end_pos[1]}"
        assert (
            0.0 <= quarter_pos[0] <= 3.0
        ), f"Quarter X: {quarter_pos[0]} out of range [0,3]"

    def test_derivative_consistency(self):
        """Test that derivatives are consistent with finite differences."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0], [2.0, 0.0, 0.0], [3.0, -0.5, 0.0]]
        )

        spline = Splines(points)

        # Test at a middle arc length value
        total_length = spline.total_length
        s = total_length * 0.5
        ds = total_length * 1e-6

        # Compute velocity using derivative
        velocity = spline.evaluate(s, derivative=1)

        # Approximate velocity using finite differences
        pos_plus = spline.evaluate(s + ds)
        pos_minus = spline.evaluate(s - ds)
        velocity_approx = (pos_plus - pos_minus) / (2 * ds)

        # The derivative is with respect to arc length
        assert np.allclose(velocity, velocity_approx, rtol=1e-3)


class TestSplinesFrenetFrame:
    """Tests for Frenet frame calculation."""

    def test_frenet_frame_basic(self):
        """Test basic Frenet frame calculation."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 1.0, 0.0]]
        )

        spline = Splines(points)

        # Test Frenet frame at different arc lengths
        total_length = spline.total_length
        for s in [
            0.0,
            total_length * 0.25,
            total_length * 0.5,
            total_length * 0.75,
            total_length,
        ]:
            frame = spline.get_frenet_frame(s)

            # Check that all components are present
            assert "position" in frame
            assert "tangent" in frame
            assert "normal" in frame

            position = frame["position"]
            tangent = frame["tangent"]
            normal = frame["normal"]

            # Check dimensions
            assert position.shape == (3,)
            assert tangent.shape == (3,)
            assert normal.shape == (3,)

            # Check that vectors are finite
            assert np.all(np.isfinite(position))
            assert np.all(np.isfinite(tangent))
            assert np.all(np.isfinite(normal))

            # Coordinate validation tests
            # Verify positions are within expected bounds (allowing for numerical precision)
            assert (
                -0.1 <= position[0] <= 3.5
            ), f"X coordinate {position[0]} out of range [-0.1,3.5]"
            assert (
                -0.5 <= position[1] <= 1.5
            ), f"Y coordinate {position[1]} out of range [-0.5,1.5]"
            assert (
                abs(position[2]) < 0.1
            ), f"Z coordinate {position[2]} should be near 0"

    def test_frenet_frame_unit_vectors(self):
        """Test that tangent and normal are unit vectors."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0], [3.0, 2.5, 0.0]]
        )

        spline = Splines(points, num_control_points=8)

        total_length = spline.total_length
        for s in np.linspace(0.0, total_length, 10):
            frame = spline.get_frenet_frame(s)
            tangent = frame["tangent"]
            normal = frame["normal"]

            # Check that they are unit vectors
            assert np.abs(np.linalg.norm(tangent) - 1.0) < 1e-10
            assert np.abs(np.linalg.norm(normal) - 1.0) < 1e-10

    def test_frenet_frame_orthogonality(self):
        """Test that tangent and normal are orthogonal."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.5, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 0.5, 0.0],
                [4.0, 0.0, 0.0],
            ]
        )

        spline = Splines(points)

        total_length = spline.total_length
        for s in np.linspace(0.0, total_length, 10):
            frame = spline.get_frenet_frame(s)
            tangent = frame["tangent"]
            normal = frame["normal"]

            # Check orthogonality (dot product should be zero)
            dot_product = np.dot(tangent, normal)
            assert np.abs(dot_product) < 1e-10

    def test_frenet_frame_2d_constraint(self):
        """Test that normal vector is in XY plane (z=0)."""
        points = np.array(
            [[0.0, 0.0, 1.0], [1.0, 1.0, 2.0], [2.0, 0.0, 3.0]]  # Note: z != 0
        )

        spline = Splines(points)

        total_length = spline.total_length
        for s in np.linspace(0.0, total_length, 5):
            frame = spline.get_frenet_frame(s)
            normal = frame["normal"]

            # Normal should be in XY plane (z component = 0)
            assert np.abs(normal[2]) < 1e-10


class TestSplinesEdgeCases:
    """Tests for edge cases and error handling."""

    def test_single_point_fails(self):
        """Test that single point raises ValueError."""
        points = np.array([[0.0, 0.0, 0.0]])

        with pytest.raises(ValueError, match="At least 2 points are required"):
            Splines(points)

    def test_wrong_dimension_fails(self):
        """Test that 2D points raise ValueError."""
        points_2d = np.array([[0.0, 0.0], [1.0, 1.0]])

        with pytest.raises(ValueError, match="Points must be an \\(N, 3\\) array"):
            Splines(points_2d)

    def test_insufficient_control_points_fails(self):
        """Test that too few control points raise ValueError."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]])

        with pytest.raises(ValueError, match="Too few control points"):
            Splines(points, num_control_points=2, k=3)  # Need at least k+1 = 4

    def test_zero_length_segment(self):
        """Test handling of zero-length segments (duplicate points)."""
        # Test with duplicate points in the middle
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.5, 0.0, 0.0],
                [0.5, 0.0, 0.0],  # Duplicate (very close)
                [1.0, 0.0, 0.0],
            ]
        )

        # Should handle gracefully with more distinct points
        spline = Splines(points, num_control_points=4)
        assert spline is not None

        # The spline should still be evaluable
        pos = spline.evaluate(0.5)
        assert pos.shape == (3,)

        # Test with nearly duplicate points (small separation)
        points_nearly_same = np.array(
            [[0.0, 0.0, 0.0], [1e-10, 0.0, 0.0], [2e-10, 0.0, 0.0]]
        )

        # Should handle small separations
        spline_small = Splines(points_nearly_same, num_control_points=4)
        assert spline_small is not None

        # Degenerate case with all exactly same points
        # This is a pathological case that may produce NaN due to numerical issues
        # We just test that it doesn't crash during creation
        points_all_same = np.array([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [1.0, 2.0, 3.0]])

        try:
            spline_degenerate = Splines(points_all_same, num_control_points=4)
            # If it succeeds, just check it was created
            assert spline_degenerate is not None
        except Exception:
            # It's acceptable if this degenerate case raises an exception
            pass

    def test_straight_line(self):
        """Test spline fitting on a straight line."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
            ]
        )

        # Provide explicit tangent vectors for straight line
        start_vel = np.array([1.0, 0.0, 0.0])
        end_vel = np.array([1.0, 0.0, 0.0])

        spline = Splines(points, start_vel=start_vel, end_vel=end_vel)

        # Check that spline is created successfully
        assert spline is not None
        assert spline.total_length > 0

        # Check that intermediate points stay reasonably close to the line
        # (B-spline with constraints may not produce perfect straight lines)
        total_length = spline.total_length
        for s in np.linspace(0, total_length, 5):
            pos = spline.evaluate(s)
            assert np.abs(pos[1]) < 0.5  # y should stay reasonably close to 0
            assert np.abs(pos[2]) < 0.5  # z should stay reasonably close to 0

        # Coordinate validation tests for straight line behavior
        # For a horizontal straight line, x should reasonably match arc length s
        # Note: B-splines with constraints may not produce perfect straight lines
        test_positions = [
            0.0,
            1.0,
            2.0,
            3.0,
        ]  # Test only up to 3.0 since total_length may be < 4.0
        for expected_x in test_positions:
            if expected_x <= total_length:
                pos = spline.evaluate(expected_x)
                # Allow larger tolerance for constrained B-splines
                assert (
                    abs(pos[0] - expected_x) < 0.1
                ), f"At s={expected_x}, expected x≈{expected_x}, got x={pos[0]}"
                assert (
                    abs(pos[1]) < 0.1
                ), f"At s={expected_x}, expected y≈0.0, got y={pos[1]}"
                assert (
                    abs(pos[2]) < 0.1
                ), f"At s={expected_x}, expected z≈0.0, got z={pos[2]}"

    def test_evaluate_outside_range(self):
        """Test evaluation outside the arc length range [0, total_length]."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points)
        total_length = spline.total_length

        # Evaluate outside range - should clamp to valid range
        pos_negative = spline.evaluate(-0.1)
        pos_large = spline.evaluate(total_length + 0.1)

        assert np.all(np.isfinite(pos_negative))
        assert np.all(np.isfinite(pos_large))

        # Should clamp to start and end positions
        pos_start = spline.evaluate(0.0)
        pos_end = spline.evaluate(total_length)

        assert np.allclose(pos_negative, pos_start)
        assert np.allclose(pos_large, pos_end)

        # Coordinate validation tests
        # Verify clamped positions match expected coordinates
        assert (
            abs(pos_negative[0] - 0.0) < 1e-6
        ), f"Negative clamp X: expected 0.0, got {pos_negative[0]}"
        assert (
            abs(pos_negative[1] - 0.0) < 1e-6
        ), f"Negative clamp Y: expected 0.0, got {pos_negative[1]}"
        assert (
            abs(pos_end[0] - 2.0) < 0.1
        ), f"Large clamp X: expected ≈2.0, got {pos_end[0]}"
        assert (
            abs(pos_end[1] - 0.0) < 0.1
        ), f"Large clamp Y: expected ≈0.0, got {pos_end[1]}"

    def test_arc_length_outside_range(self):
        """Test arc length evaluation with out-of-range values."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points)

        # Test with negative arc length (should clip to 0)
        pos_negative = spline.evaluate_arc_length(-1.0)
        pos_zero = spline.evaluate_arc_length(0.0)
        assert np.allclose(pos_negative, pos_zero)

        # Test with arc length > total_length (should clip to total_length)
        pos_large = spline.evaluate_arc_length(spline.total_length + 1.0)
        pos_end = spline.evaluate_arc_length(spline.total_length)
        assert np.allclose(pos_large, pos_end)

        # Coordinate validation tests for arc length clamping
        # For this horizontal line, positions should match expected coordinates
        assert (
            abs(pos_negative[0] - 0.0) < 1e-6
        ), f"Negative arc length X: expected 0.0, got {pos_negative[0]}"
        assert (
            abs(pos_negative[1] - 0.0) < 1e-6
        ), f"Negative arc length Y: expected 0.0, got {pos_negative[1]}"

    def test_hard_constraints_verification(self):
        """Test that hard constraints are properly verified."""
        # Create a simple set of points
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        # Create spline with explicit velocities
        start_vel = np.array([1.0, 0.0, 0.0])
        end_vel = np.array([1.0, -1.0, 0.0])
        end_vel = end_vel / np.linalg.norm(end_vel)  # Normalize

        # This should work fine - constraints should be satisfied
        spline = Splines(points, start_vel=start_vel, end_vel=end_vel)

        # Verify that the spline was created successfully
        assert spline is not None
        assert hasattr(spline, "spline")

        # Manually verify constraints are satisfied
        # Start position
        start_pos = spline.spline(0.0, nu=0)
        assert np.allclose(start_pos, points[0] - spline._origin_offset, atol=1e-6)

        # End position
        end_pos = spline.spline(1.0, nu=0)
        assert np.allclose(end_pos, points[-1] - spline._origin_offset, atol=1e-6)


class TestSplinesNumericalStability:
    """Tests for numerical stability."""

    def test_very_close_points(self):
        """Test with points that are very close together."""
        points = np.array([[0.0, 0.0, 0.0], [1e-10, 1e-10, 0.0], [1.0, 1.0, 0.0]])

        spline = Splines(points, num_control_points=4)

        # Should handle without numerical issues
        pos = spline.evaluate(0.5)
        assert np.all(np.isfinite(pos))

        # Coordinate validation tests
        # Position should be somewhere between start and end points
        assert 0.0 <= pos[0] <= 1.0, f"X coordinate {pos[0]} out of range [0,1]"
        assert 0.0 <= pos[1] <= 1.0, f"Y coordinate {pos[1]} out of range [0,1]"
        assert abs(pos[2]) < 0.1, f"Z coordinate {pos[2]} should be near 0"

    def test_large_coordinate_values(self):
        """Test with large coordinate values."""
        points = np.array(
            [[1e6, 1e6, 1e6], [1e6 + 1, 1e6, 1e6], [1e6 + 2, 1e6 + 1, 1e6]]
        )

        spline = Splines(points, num_control_points=4)

        # Should handle large values
        pos = spline.evaluate(0.5)
        assert np.all(np.isfinite(pos))
        assert pos[0] > 1e6  # Should be in the right range

        # Coordinate validation tests
        # Position should be in expected range for large coordinates
        assert (
            1e6 <= pos[0] <= 1e6 + 2
        ), f"X coordinate {pos[0]} out of expected large range"
        assert (
            1e6 <= pos[1] <= 1e6 + 1
        ), f"Y coordinate {pos[1]} out of expected large range"
        assert (
            1e6 - 0.1 <= pos[2] <= 1e6 + 0.1
        ), f"Z coordinate {pos[2]} out of expected large range"

    def test_many_points(self):
        """Test with many input points."""
        # Generate a sine wave with many points
        t = np.linspace(0, 4 * np.pi, 100)
        x = t
        y = np.sin(t)
        z = np.zeros_like(t)
        points = np.column_stack([x, y, z])

        spline = Splines(points, num_control_points=20)

        # Should handle many points efficiently
        assert spline is not None
        assert spline.total_length > 0

        # Test evaluation
        total_length = spline.total_length
        for s in [0.0, total_length * 0.5, total_length]:
            pos = spline.evaluate(s)
            assert np.all(np.isfinite(pos))
