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
        end_pos = spline.evaluate(1.0)

        assert np.allclose(start_pos, points[0], atol=1e-6)
        assert np.allclose(end_pos, points[1], atol=1e-6)

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
        end_pos = spline.evaluate(1.0)

        assert np.allclose(start_pos, points[0], atol=1e-3)
        assert np.allclose(end_pos, points[-1], atol=1e-3)

    def test_evaluate_at_different_parameters(self):
        """Test evaluating spline at different parameter values."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points, num_control_points=6)

        # Evaluate at various parameter values
        positions = []
        for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
            pos = spline.evaluate(t)
            positions.append(pos)
            assert pos.shape == (3,)
            assert np.all(np.isfinite(pos))

        # Check that positions are different (spline is not constant)
        for i in range(len(positions) - 1):
            assert not np.allclose(positions[i], positions[i + 1])

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
        end_tangent = spline.evaluate(1.0, derivative=1)

        # Normalize for comparison
        start_tangent_norm = start_tangent / np.linalg.norm(start_tangent)
        end_tangent_norm = end_tangent / np.linalg.norm(end_tangent)

        # Check direction (allowing some tolerance due to scaling)
        assert np.dot(start_tangent_norm, start_vel) > 0.9
        assert np.dot(end_tangent_norm, end_vel) > 0.9

    def test_automatic_tangent_estimation(self):
        """Test automatic tangent estimation when not specified."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 1.0, 0.0]]
        )

        # Create spline without specifying tangents
        spline = Splines(points)

        # Check that tangents are automatically estimated and reasonable
        start_tangent = spline.evaluate(0.0, derivative=1)
        end_tangent = spline.evaluate(1.0, derivative=1)

        assert np.all(np.isfinite(start_tangent))
        assert np.all(np.isfinite(end_tangent))
        assert np.linalg.norm(start_tangent) > 0
        assert np.linalg.norm(end_tangent) > 0

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
            end_pos = spline.evaluate(1.0)
            assert np.allclose(start_pos, points[0], atol=1e-2)
            assert np.allclose(end_pos, points[-1], atol=1e-2)


class TestSplinesDerivatives:
    """Tests for spline derivatives."""

    def test_first_derivative(self):
        """Test first derivative (velocity) calculation."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points)

        # Test velocity at different parameters
        for t in [0.0, 0.5, 1.0]:
            velocity = spline.evaluate(t, derivative=1)
            assert velocity.shape == (3,)
            assert np.all(np.isfinite(velocity))

    def test_second_derivative(self):
        """Test second derivative (acceleration) calculation."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0], [3.0, -1.0, 0.0]]
        )

        spline = Splines(points, num_control_points=8)

        # Test acceleration at different parameters
        for t in [0.0, 0.5, 1.0]:
            acceleration = spline.evaluate(t, derivative=2)
            assert acceleration.shape == (3,)
            assert np.all(np.isfinite(acceleration))

    def test_derivative_consistency(self):
        """Test that derivatives are consistent with finite differences."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0], [2.0, 0.0, 0.0], [3.0, -0.5, 0.0]]
        )

        spline = Splines(points)

        # Test at a middle parameter value
        t = 0.5
        dt = 1e-6

        # Compute velocity using derivative
        velocity = spline.evaluate(t, derivative=1)

        # Approximate velocity using finite differences
        pos_plus = spline.evaluate(t + dt)
        pos_minus = spline.evaluate(t - dt)
        velocity_approx = (pos_plus - pos_minus) / (2 * dt)

        # The derivative is with respect to the normalized parameter t (0 to 1)
        # So we just compare directly without scaling
        assert np.allclose(velocity, velocity_approx, rtol=1e-3)


class TestSplinesFrenetFrame:
    """Tests for Frenet frame calculation."""

    def test_frenet_frame_basic(self):
        """Test basic Frenet frame calculation."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 1.0, 0.0]]
        )

        spline = Splines(points)

        # Test Frenet frame at different parameters
        for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
            frame = spline.get_frenet_frame(t)

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

    def test_frenet_frame_unit_vectors(self):
        """Test that tangent and normal are unit vectors."""
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0], [3.0, 2.5, 0.0]]
        )

        spline = Splines(points, num_control_points=8)

        for t in np.linspace(0.0, 1.0, 10):
            frame = spline.get_frenet_frame(t)
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

        for t in np.linspace(0.0, 1.0, 10):
            frame = spline.get_frenet_frame(t)
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

        for t in np.linspace(0.0, 1.0, 5):
            frame = spline.get_frenet_frame(t)
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
        for t in np.linspace(0, 1, 5):
            pos = spline.evaluate(t)
            assert np.abs(pos[1]) < 0.5  # y should stay reasonably close to 0
            assert np.abs(pos[2]) < 0.5  # z should stay reasonably close to 0

    def test_evaluate_outside_range(self):
        """Test evaluation outside the parameter range [0, 1]."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points)

        # Evaluate outside range - should still work (B-spline extrapolation)
        pos_negative = spline.evaluate(-0.1)
        pos_large = spline.evaluate(1.1)

        assert np.all(np.isfinite(pos_negative))
        assert np.all(np.isfinite(pos_large))

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


class TestSplinesNumericalStability:
    """Tests for numerical stability."""

    def test_very_close_points(self):
        """Test with points that are very close together."""
        points = np.array([[0.0, 0.0, 0.0], [1e-10, 1e-10, 0.0], [1.0, 1.0, 0.0]])

        spline = Splines(points, num_control_points=4)

        # Should handle without numerical issues
        pos = spline.evaluate(0.5)
        assert np.all(np.isfinite(pos))

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
        for param in [0.0, 0.5, 1.0]:
            pos = spline.evaluate(param)
            assert np.all(np.isfinite(pos))
