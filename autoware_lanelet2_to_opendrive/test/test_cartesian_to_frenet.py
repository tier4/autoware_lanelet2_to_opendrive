"""Tests for Cartesian to Frenet coordinate conversion."""

import numpy as np
from autoware_lanelet2_to_opendrive.spline import Splines


class TestCartesianToFrenet:
    """Test suite for cartesian_to_frenet method."""

    def test_straight_line_on_spline(self):
        """Test conversion for a point on a straight line spline."""
        # Create a straight line spline from (0,0,0) to (100,0,0)
        points = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        spline = Splines(points, num_control_points=4)

        # Test point on the spline
        x, y, z = 50.0, 0.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should be at arc length ~50, lateral offset ~0
        assert abs(s - 50.0) < 1.0, f"Expected s≈50, got {s}"
        assert abs(d) < 0.5, f"Expected d≈0, got {d}"

    def test_straight_line_left_of_spline(self):
        """Test conversion for a point to the left of a straight line spline."""
        # Create a straight line spline from (0,0,0) to (100,0,0)
        points = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        spline = Splines(points, num_control_points=4)

        # Test point to the left (positive y, positive d)
        x, y, z = 50.0, 5.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should be at arc length ~50, lateral offset ~5 (positive = left)
        assert abs(s - 50.0) < 1.0, f"Expected s≈50, got {s}"
        assert 4.0 < d < 6.0, f"Expected d≈5 (positive for left), got {d}"

    def test_straight_line_right_of_spline(self):
        """Test conversion for a point to the right of a straight line spline."""
        # Create a straight line spline from (0,0,0) to (100,0,0)
        points = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        spline = Splines(points, num_control_points=4)

        # Test point to the right (negative y, negative d)
        x, y, z = 50.0, -5.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should be at arc length ~50, lateral offset ~-5 (negative = right)
        assert abs(s - 50.0) < 1.0, f"Expected s≈50, got {s}"
        assert -6.0 < d < -4.0, f"Expected d≈-5 (negative for right), got {d}"

    def test_curved_spline_inner_side(self):
        """Test conversion for a point on the inner side of a curve."""
        # Create an S-curve
        wx = np.array([0.0, 20.0, 40.0, 60.0, 80.0])
        wy = np.array([0.0, 0.0, 20.0, 20.0, 40.0])
        wz = np.zeros_like(wx)
        points = np.column_stack([wx, wy, wz])

        spline = Splines(points, num_control_points=10)

        # Test point near the middle of the curve (around x=40, y=15)
        # This should be on the right side (inner curve) of the spline
        x, y, z = 40.0, 15.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Arc length should be somewhere in the middle
        total_length = spline.total_length
        assert (
            0.3 * total_length < s < 0.7 * total_length
        ), f"Expected s in middle range, got {s} (total={total_length})"

        # Should have some lateral offset (right side of curve = negative)
        assert d < 0, f"Expected negative d (right side), got {d}"

    def test_curved_spline_outer_side(self):
        """Test conversion for a point on the outer side of a curve."""
        # Create an S-curve
        wx = np.array([0.0, 20.0, 40.0, 60.0, 80.0])
        wy = np.array([0.0, 0.0, 20.0, 20.0, 40.0])
        wz = np.zeros_like(wx)
        points = np.column_stack([wx, wy, wz])

        spline = Splines(points, num_control_points=10)

        # Test point on the outer side (left) of the curve
        x, y, z = 65.0, 25.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Arc length should be in the latter part
        total_length = spline.total_length
        assert (
            0.5 * total_length < s < total_length
        ), f"Expected s in latter range, got {s} (total={total_length})"

        # Should have positive lateral offset (left side)
        assert d > 0, f"Expected positive d (left side), got {d}"

    def test_start_point(self):
        """Test conversion at the start point of the spline."""
        points = np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        spline = Splines(points, num_control_points=6)

        # Test at start point
        x, y, z = 0.0, 0.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should be at arc length 0
        assert s < 1.0, f"Expected s≈0 at start, got {s}"
        assert abs(d) < 0.5, f"Expected d≈0 at start, got {d}"

    def test_end_point(self):
        """Test conversion at the end point of the spline."""
        points = np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        spline = Splines(points, num_control_points=6)

        total_length = spline.total_length

        # Test at end point
        x, y, z = 100.0, 0.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should be at arc length ≈ total_length
        assert abs(s - total_length) < 1.0, f"Expected s≈{total_length} at end, got {s}"
        assert abs(d) < 0.5, f"Expected d≈0 at end, got {d}"

    def test_2d_default_z(self):
        """Test that default z=0.0 works for 2D applications."""
        # Create a 2D spline
        points = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        spline = Splines(points, num_control_points=4)

        # Test without specifying z (should default to 0.0)
        x, y = 50.0, 5.0
        s, d = spline.cartesian_to_frenet(x, y)

        # Should work correctly
        assert abs(s - 50.0) < 1.0, f"Expected s≈50, got {s}"
        assert 4.0 < d < 6.0, f"Expected d≈5, got {d}"

    def test_3d_spline(self):
        """Test conversion with a 3D spline."""
        # Create a 3D helix-like spline
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [10.0, 10.0, 5.0],
                [20.0, 0.0, 10.0],
                [30.0, -10.0, 15.0],
            ]
        )
        spline = Splines(points, num_control_points=8)

        # Test a point near the middle
        x, y, z = 15.0, 5.0, 7.5
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should return valid s and d values
        total_length = spline.total_length
        assert 0 <= s <= total_length, f"s={s} should be in range [0, {total_length}]"
        # d can be any value depending on how far the point is from the spline
        assert isinstance(d, (float, np.floating)), "d should be a float"

    def test_large_coordinates(self):
        """Test conversion with large coordinate values (numerical stability)."""
        # Create a spline with large coordinates (e.g., UTM coordinates)
        offset = 500000.0
        points = np.array(
            [
                [offset, offset, 0.0],
                [offset + 100.0, offset, 0.0],
            ]
        )
        spline = Splines(points, num_control_points=4)

        # Test point with large coordinates
        x, y, z = offset + 50.0, offset + 5.0, 0.0
        s, d = spline.cartesian_to_frenet(x, y, z)

        # Should still work correctly
        assert abs(s - 50.0) < 1.0, f"Expected s≈50, got {s}"
        assert 4.0 < d < 6.0, f"Expected d≈5, got {d}"
