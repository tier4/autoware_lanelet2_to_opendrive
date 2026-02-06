"""Tests to verify boundary velocity vector calculation hypothesis."""

import numpy as np
import lanelet2
from autoware_lanelet2_to_opendrive.spline import Splines
from autoware_lanelet2_to_opendrive.centerline import (
    _get_boundary_start_vel,
    _get_boundary_end_vel,
    _calculate_centerline_velocity_vector,
)


def create_test_lanelet_with_curved_boundaries():
    """Create a test lanelet where left and right boundaries have different curvatures."""
    # Left boundary: slightly curved to the left
    left_points = [
        lanelet2.core.Point3d(1, 0.0, 0.0, 0.0),
        lanelet2.core.Point3d(2, 0.1, 1.0, 0.0),
        lanelet2.core.Point3d(3, 0.2, 2.0, 0.0),
        lanelet2.core.Point3d(4, 0.3, 3.0, 0.0),
    ]
    left_bound = lanelet2.core.LineString3d(1, left_points)

    # Right boundary: slightly curved to the right
    right_points = [
        lanelet2.core.Point3d(5, 3.0, 0.0, 0.0),
        lanelet2.core.Point3d(6, 2.9, 1.0, 0.0),
        lanelet2.core.Point3d(7, 2.8, 2.0, 0.0),
        lanelet2.core.Point3d(8, 2.7, 3.0, 0.0),
    ]
    right_bound = lanelet2.core.LineString3d(2, right_points)

    return lanelet2.core.Lanelet(1, left_bound, right_bound)


def test_boundary_velocity_discrepancy():
    """
    Test that current implementation creates different velocity vectors for left and right boundaries.

    This test verifies the hypothesis that using adjacent points within each boundary
    independently causes velocity vector misalignment at endpoints.
    """
    lanelet = create_test_lanelet_with_curved_boundaries()

    # Current implementation: Calculate velocity from adjacent points in each boundary
    left_start_vel = _get_boundary_start_vel(lanelet.leftBound)[:2]
    right_start_vel = _get_boundary_start_vel(lanelet.rightBound)[:2]

    # Verify that the vectors are different (demonstrating the problem)
    dot_product = np.dot(left_start_vel, right_start_vel)
    angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
    angle_deg = np.degrees(angle_rad)

    # With curved boundaries, the angle should be significantly different from 0
    assert (
        angle_deg > 5.0
    ), f"Expected significant angle difference, but got {angle_deg:.2f}°"

    # The vectors should have noticeable difference
    vector_diff = np.linalg.norm(left_start_vel - right_start_vel)
    assert (
        vector_diff > 0.05
    ), f"Expected vector difference > 0.05, got {vector_diff:.6f}"


def test_centerline_velocity_consistency():
    """
    Test that centerline velocity calculation produces consistent vectors.

    This test verifies that the correct implementation (using perpendicular to
    left-right connection) produces the same velocity vector for both boundaries.
    """
    lanelet = create_test_lanelet_with_curved_boundaries()

    # Correct implementation: Calculate velocity perpendicular to left-right connection
    correct_start_vel = _calculate_centerline_velocity_vector(lanelet, at_start=True)[
        :2
    ]

    # The same vector should be used for both left and right boundaries
    # Verify it's perpendicular to the left-right segment
    left_point = np.array([lanelet.leftBound[0].x, lanelet.leftBound[0].y])
    right_point = np.array([lanelet.rightBound[0].x, lanelet.rightBound[0].y])
    segment = right_point - left_point

    # Dot product should be close to 0 (perpendicular)
    dot_product = np.dot(correct_start_vel, segment)
    assert (
        abs(dot_product) < 1e-10
    ), f"Expected perpendicular vectors, got dot={dot_product}"

    # Verify the vector is normalized
    length = np.linalg.norm(correct_start_vel)
    assert abs(length - 1.0) < 1e-10, f"Expected unit vector, got length={length}"


def test_spline_tangent_alignment_with_correct_velocity():
    """
    Test that splines created with correct velocity vectors have aligned tangents.

    This test verifies that when both boundaries use the same velocity vector
    (perpendicular to left-right connection), the resulting splines have
    aligned tangent directions at endpoints.
    """
    lanelet = create_test_lanelet_with_curved_boundaries()

    # Extract boundary points
    left_points = np.array([[p.x, p.y] for p in lanelet.leftBound])
    right_points = np.array([[p.x, p.y] for p in lanelet.rightBound])

    # Correct velocity vectors (same for both boundaries)
    correct_start_vel = _calculate_centerline_velocity_vector(lanelet, at_start=True)[
        :2
    ]
    correct_end_vel = _calculate_centerline_velocity_vector(lanelet, at_start=False)[:2]

    # Create splines with correct velocity vectors
    left_spline = Splines(
        left_points,
        start_vel=correct_start_vel,
        end_vel=correct_end_vel,
        num_control_points=6,
    )
    right_spline = Splines(
        right_points,
        start_vel=correct_start_vel,
        end_vel=correct_end_vel,
        num_control_points=6,
    )

    # Evaluate tangents at start point
    left_tangent = left_spline.evaluate(0.0, derivative=1)[:2]
    left_tangent = left_tangent / np.linalg.norm(left_tangent)

    right_tangent = right_spline.evaluate(0.0, derivative=1)[:2]
    right_tangent = right_tangent / np.linalg.norm(right_tangent)

    # Calculate angle between tangents
    dot_product = np.dot(left_tangent, right_tangent)
    angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
    angle_deg = np.degrees(angle_rad)

    # Tangents should be very close (angle < 1 degree)
    assert (
        angle_deg < 1.0
    ), f"Expected aligned tangents (angle < 1°), but got {angle_deg:.2f}°"


def test_spline_tangent_misalignment_with_wrong_velocity():
    """
    Test that splines created with wrong velocity vectors have misaligned tangents.

    This test demonstrates that when each boundary uses its own velocity vector
    (from adjacent points), the resulting splines have different tangent
    directions at endpoints, causing gaps.
    """
    lanelet = create_test_lanelet_with_curved_boundaries()

    # Extract boundary points
    left_points = np.array([[p.x, p.y] for p in lanelet.leftBound])
    right_points = np.array([[p.x, p.y] for p in lanelet.rightBound])

    # Wrong velocity vectors (different for each boundary)
    left_start_vel = _get_boundary_start_vel(lanelet.leftBound)[:2]
    left_end_vel = _get_boundary_end_vel(lanelet.leftBound)[:2]
    right_start_vel = _get_boundary_start_vel(lanelet.rightBound)[:2]
    right_end_vel = _get_boundary_end_vel(lanelet.rightBound)[:2]

    # Create splines with wrong velocity vectors
    left_spline = Splines(
        left_points,
        start_vel=left_start_vel,
        end_vel=left_end_vel,
        num_control_points=6,
    )
    right_spline = Splines(
        right_points,
        start_vel=right_start_vel,
        end_vel=right_end_vel,
        num_control_points=6,
    )

    # Evaluate tangents at start point
    left_tangent = left_spline.evaluate(0.0, derivative=1)[:2]
    left_tangent = left_tangent / np.linalg.norm(left_tangent)

    right_tangent = right_spline.evaluate(0.0, derivative=1)[:2]
    right_tangent = right_tangent / np.linalg.norm(right_tangent)

    # Calculate angle between tangents
    dot_product = np.dot(left_tangent, right_tangent)
    angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
    angle_deg = np.degrees(angle_rad)

    # Tangents should be significantly misaligned (angle > 5 degrees)
    assert (
        angle_deg > 5.0
    ), f"Expected misaligned tangents (angle > 5°), but got {angle_deg:.2f}°"


def test_endpoint_velocity_comparison():
    """
    Compare velocity vectors at both start and end points for all methods.

    This comprehensive test verifies the hypothesis for both start and end points.
    """
    lanelet = create_test_lanelet_with_curved_boundaries()

    # Test start point
    left_start_wrong = _get_boundary_start_vel(lanelet.leftBound)[:2]
    right_start_wrong = _get_boundary_start_vel(lanelet.rightBound)[:2]
    correct_start = _calculate_centerline_velocity_vector(lanelet, at_start=True)[:2]

    # Start: Wrong method produces different vectors
    wrong_diff_start = np.linalg.norm(left_start_wrong - right_start_wrong)
    assert (
        wrong_diff_start > 0.05
    ), "Start: Wrong method should produce different vectors"

    # Start: Correct method is perpendicular to left-right segment
    left_point = np.array([lanelet.leftBound[0].x, lanelet.leftBound[0].y])
    right_point = np.array([lanelet.rightBound[0].x, lanelet.rightBound[0].y])
    segment_start = right_point - left_point
    assert (
        abs(np.dot(correct_start, segment_start)) < 1e-10
    ), "Start: Should be perpendicular"

    # Test end point
    left_end_wrong = _get_boundary_end_vel(lanelet.leftBound)[:2]
    right_end_wrong = _get_boundary_end_vel(lanelet.rightBound)[:2]
    correct_end = _calculate_centerline_velocity_vector(lanelet, at_start=False)[:2]

    # End: Wrong method produces different vectors
    wrong_diff_end = np.linalg.norm(left_end_wrong - right_end_wrong)
    assert wrong_diff_end > 0.05, "End: Wrong method should produce different vectors"

    # End: Correct method is perpendicular to left-right segment
    left_point_end = np.array([lanelet.leftBound[-1].x, lanelet.leftBound[-1].y])
    right_point_end = np.array([lanelet.rightBound[-1].x, lanelet.rightBound[-1].y])
    segment_end = right_point_end - left_point_end
    assert abs(np.dot(correct_end, segment_end)) < 1e-10, "End: Should be perpendicular"
