import numpy as np
import lanelet2
from typing import Set
from .geometry import (
    point_to_line_segment_distance,
)
from .spline import Splines
from .util import sort_adjacent_groups


class AsymmetryLaneletException(Exception):
    """Exception raised when a lanelet has asymmetric left and right widths."""

    pass


def _get_boundary_start_vel(boundary) -> np.ndarray:
    """
    Calculate start velocity vector for a boundary spline.

    Args:
        boundary: List of points representing a boundary

    Returns:
        3D velocity vector along the boundary direction
    """
    if len(boundary) < 2:
        return np.array([1.0, 0.0, 0.0])  # Default direction

    # Calculate direction from first to second point
    start_point = np.array([boundary[0].x, boundary[0].y, boundary[0].z])
    next_point = np.array([boundary[1].x, boundary[1].y, boundary[1].z])

    direction = next_point - start_point
    length = np.linalg.norm(direction)

    if length < 1e-10:
        return np.array([1.0, 0.0, 0.0])  # Fallback

    return direction / length


def _get_boundary_end_vel(boundary) -> np.ndarray:
    """
    Calculate end velocity vector for a boundary spline.

    Args:
        boundary: List of points representing a boundary

    Returns:
        3D velocity vector along the boundary direction
    """
    if len(boundary) < 2:
        return np.array([1.0, 0.0, 0.0])  # Default direction

    # Calculate direction from second-to-last to last point
    prev_point = np.array([boundary[-2].x, boundary[-2].y, boundary[-2].z])
    end_point = np.array([boundary[-1].x, boundary[-1].y, boundary[-1].z])

    direction = end_point - prev_point
    length = np.linalg.norm(direction)

    if length < 1e-10:
        return np.array([1.0, 0.0, 0.0])  # Fallback

    return direction / length


def _get_start_vel(lanelet: lanelet2.core.Lanelet) -> np.ndarray:
    """
    Calculate start velocity vector for centerline spline from lanelet boundaries.

    Args:
        lanelet: A Lanelet2 lanelet object

    Returns:
        3D velocity vector perpendicular to the line connecting first points of left and right boundaries
    """
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 1 or len(right_bound) < 1:
        raise ValueError(
            "Lanelet must have at least 1 point in both left and right boundaries"
        )

    # Get first points of boundaries
    left_first = np.array([left_bound[0].x, left_bound[0].y])
    right_first = np.array([right_bound[0].x, right_bound[0].y])

    # Calculate line segment connecting left and right boundaries
    segment = right_first - left_first  # Vector from left to right

    # Calculate perpendicular vector (2D) pointing forward along the lanelet
    # Rotate 90 degrees counter-clockwise: (x, y) -> (-y, x)
    perp_2d = np.array([-segment[1], segment[0]])

    # Normalize to unit vector
    length = np.linalg.norm(perp_2d)
    if length < 1e-10:
        # Fallback to default direction if boundaries are parallel
        perp_2d = np.array([1.0, 0.0])
    else:
        perp_2d = perp_2d / length

    # Convert to 3D by adding z=0
    return np.array([perp_2d[0], perp_2d[1], 0.0])


def _get_end_vel(lanelet: lanelet2.core.Lanelet) -> np.ndarray:
    """
    Calculate end velocity vector for centerline spline from lanelet boundaries.

    Args:
        lanelet: A Lanelet2 lanelet object

    Returns:
        3D velocity vector perpendicular to the line connecting last points of left and right boundaries
    """
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 1 or len(right_bound) < 1:
        raise ValueError(
            "Lanelet must have at least 1 point in both left and right boundaries"
        )

    # Get last points of boundaries
    left_last = np.array([left_bound[-1].x, left_bound[-1].y])
    right_last = np.array([right_bound[-1].x, right_bound[-1].y])

    # Calculate line segment connecting left and right boundaries
    segment = right_last - left_last  # Vector from left to right

    # Calculate perpendicular vector (2D) pointing forward along the lanelet
    # Rotate 90 degrees counter-clockwise: (x, y) -> (-y, x)
    perp_2d = np.array([-segment[1], segment[0]])

    # Normalize to unit vector
    length = np.linalg.norm(perp_2d)
    if length < 1e-10:
        # Fallback to default direction if boundaries are parallel
        perp_2d = np.array([1.0, 0.0])
    else:
        perp_2d = perp_2d / length

    # Convert to 3D by adding z=0
    return np.array([perp_2d[0], perp_2d[1], 0.0])


def extract_centerline_as_spline(
    lanelet: lanelet2.core.Lanelet, num_control_points: int = 50
) -> Splines:
    """
    Extract centerline from a Lanelet using midpoints between left and right borders.

    Uses line segment representation of borders to calculate centerline points
    by interpolating along both borders using normalized coordinates.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_control_points: Number of control points for B-spline interpolation

    Returns:
        Splines object that can be evaluated using arc length
    """
    # Get raw boundary points
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 2 or len(right_bound) < 2:
        raise ValueError("Both boundaries must have at least 2 points")

    # Convert to numpy arrays
    left_points = np.array([[p.x, p.y, p.z] for p in left_bound])
    right_points = np.array([[p.x, p.y, p.z] for p in right_bound])

    # Calculate cumulative arc lengths for both boundaries
    left_dists = np.linalg.norm(np.diff(left_points, axis=0), axis=1)
    left_cumulative = np.concatenate(([0], np.cumsum(left_dists)))
    left_total_length = left_cumulative[-1]

    right_dists = np.linalg.norm(np.diff(right_points, axis=0), axis=1)
    right_cumulative = np.concatenate(([0], np.cumsum(right_dists)))
    right_total_length = right_cumulative[-1]

    # Number of sample points for centerline
    num_samples = max(20, num_control_points * 2)

    centerline_points = []
    for i in range(num_samples):
        # Normalized coordinate from 0 to 1
        t_normalized = i / (num_samples - 1) if num_samples > 1 else 0.0

        # Convert to arc length for each boundary
        left_s = t_normalized * left_total_length
        right_s = t_normalized * right_total_length

        # Interpolate on left boundary line segments
        left_point = _interpolate_on_line_segments(left_points, left_cumulative, left_s)

        # Interpolate on right boundary line segments
        right_point = _interpolate_on_line_segments(
            right_points, right_cumulative, right_s
        )

        # Calculate midpoint
        midpoint = (left_point + right_point) / 2.0
        centerline_points.append(midpoint)

    centerline_points = np.array(centerline_points)

    # Create B-spline with constrained fitting
    return Splines(
        centerline_points,
        start_vel=_get_start_vel(lanelet),
        end_vel=_get_end_vel(lanelet),
        num_control_points=num_control_points,
    )


def _interpolate_on_line_segments(
    points: np.ndarray, cumulative_lengths: np.ndarray, s: float
) -> np.ndarray:
    """
    Interpolate a point along line segments at given arc length.

    Args:
        points: Array of points defining the line segments (N x 3)
        cumulative_lengths: Cumulative arc lengths at each point (N,)
        s: Target arc length for interpolation

    Returns:
        Interpolated 3D point
    """
    # Clamp s to valid range
    s = np.clip(s, 0.0, cumulative_lengths[-1])

    # Find which segment contains the target arc length
    segment_idx = np.searchsorted(cumulative_lengths, s) - 1
    segment_idx = np.clip(segment_idx, 0, len(points) - 2)

    # Get segment endpoints
    p1 = points[segment_idx]
    p2 = points[segment_idx + 1]

    # Get arc lengths at segment endpoints
    s1 = cumulative_lengths[segment_idx]
    s2 = cumulative_lengths[segment_idx + 1]

    # Avoid division by zero for zero-length segments
    if abs(s2 - s1) < 1e-10:
        return p1

    # Linear interpolation within the segment
    t = (s - s1) / (s2 - s1)
    return p1 + t * (p2 - p1)


def extract_border_from_spline(
    lanelet: lanelet2.core.Lanelet, border: str, num_control_points: int = 50
) -> Splines:
    """
    Extract border line from a Lanelet and return as B-spline with arc length parameterization.

    Args:
        lanelet: A Lanelet2 lanelet object
        border: Border specification - "left" or "right"
        num_control_points: Number of control points for B-spline interpolation

    Returns:
        Splines object that can be evaluated using arc length

    Raises:
        ValueError: If border is not "left" or "right", or if insufficient points
    """
    if border not in ["left", "right"]:
        raise ValueError(f"Invalid border: {border}. Must be 'left' or 'right'")

    # Get the appropriate boundary
    if border == "left":
        boundary = lanelet.leftBound
    else:  # border == "right"
        boundary = lanelet.rightBound

    if len(boundary) < 2:
        raise ValueError(
            f"Lanelet must have at least 2 points in its {border} boundary"
        )

    # Extract points from the boundary
    points = []
    for point in boundary:
        points.append([point.x, point.y, point.z])

    points = np.array(points)

    # Create B-spline with constrained fitting
    return Splines(
        points,
        start_vel=_get_boundary_start_vel(boundary),
        end_vel=_get_boundary_end_vel(boundary),
        num_control_points=num_control_points,
    )


def estimate_lanelet_width_as_spline(
    lanelet: lanelet2.core.Lanelet,
    num_samples: int = 20,
    num_control_points: int = 50,
    reference: str = "center_line",
) -> Splines:
    """
    Estimate lanelet total width along its centerline or left boundary using Frenet coordinates.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_samples: Number of sample points along the reference line
        num_control_points: Number of control points for B-spline interpolation
        reference: Reference line for width calculation - "center_line" or "left_bound"

    Returns:
        Splines object representing the total width (left + right distances)
    """
    if reference not in ["center_line", "left_bound"]:
        raise ValueError(
            f"Invalid reference: {reference}. Must be 'center_line' or 'left_bound'"
        )

    # Get arc length parameterized reference spline based on mode
    if reference == "center_line":
        length_based_spline = extract_centerline_as_spline(lanelet, num_control_points)
    else:  # left_bound
        # Extract left boundary points directly
        left_bound = lanelet.leftBound
        if len(left_bound) < 2:
            raise ValueError("Lanelet must have at least 2 points in its left bound")

        points = []
        for point in left_bound:
            points.append([point.x, point.y, point.z])

        length_based_spline = Splines(
            np.array(points), num_control_points=num_control_points
        )

    total_length = length_based_spline.total_length

    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    left_bound_points = np.array([[p.x, p.y, p.z] for p in left_bound])
    right_bound_points = np.array([[p.x, p.y, p.z] for p in right_bound])

    # Create length-based sampling points
    length_values = np.linspace(0, total_length, num_samples)
    total_widths = []

    for length in length_values:
        # Use Frenet coordinate calculation from the spline class
        frenet_frame = length_based_spline.get_frenet_frame(length)
        reference_point = frenet_frame["position"]

        if reference == "center_line":
            # Find closest distance to left boundary (use simple point-to-line distance)
            min_left_dist = float("inf")
            for i in range(len(left_bound_points) - 1):
                seg_start = left_bound_points[i]
                seg_end = left_bound_points[i + 1]

                dist = point_to_line_segment_distance(
                    reference_point, seg_start, seg_end, None
                )
                if dist is not None and dist < min_left_dist:
                    min_left_dist = dist

            # Find closest distance to right boundary
            min_right_dist = float("inf")
            for i in range(len(right_bound_points) - 1):
                seg_start = right_bound_points[i]
                seg_end = right_bound_points[i + 1]

                dist = point_to_line_segment_distance(
                    reference_point, seg_start, seg_end, None
                )
                if dist is not None and dist < min_right_dist:
                    min_right_dist = dist

            left_width = min_left_dist if min_left_dist != float("inf") else 0.0
            right_width = min_right_dist if min_right_dist != float("inf") else 0.0

            # Check for asymmetry between left and right widths only for center_line reference
            # Threshold of 0.3m is hardcoded as a parameter for detecting asymmetric lanelets
            ASYMMETRY_THRESHOLD = 0.3  # meters
            if abs(left_width - right_width) > ASYMMETRY_THRESHOLD:
                raise AsymmetryLaneletException(
                    f"Lanelet {lanelet.id} has asymmetric widths: "
                    f"left={left_width:.2f}m, right={right_width:.2f}m, "
                    f"difference={abs(left_width - right_width):.2f}m > {ASYMMETRY_THRESHOLD}m threshold"
                )

        else:  # reference == "left_bound"
            # When using left boundary as reference, left width is always 0
            left_width = 0.0

            # Find closest distance to right boundary only
            min_right_dist = float("inf")
            for i in range(len(right_bound_points) - 1):
                seg_start = right_bound_points[i]
                seg_end = right_bound_points[i + 1]

                dist = point_to_line_segment_distance(
                    reference_point, seg_start, seg_end, None
                )
                if dist is not None and dist < min_right_dist:
                    min_right_dist = dist

            right_width = min_right_dist if min_right_dist != float("inf") else 0.0
            # No asymmetry check needed for left_bound reference mode

        total_widths.append(left_width + right_width)

    # Create 1D spline for total width values
    # Create points as [[length0, width0, 0], [length1, width1, 0], ...]
    width_points = np.column_stack(
        [length_values, total_widths, np.zeros(len(total_widths))]
    )

    return Splines(
        width_points, num_control_points=min(num_control_points, len(width_points))
    )


def extract_centerline_as_spline_from_two_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    two_lanelets: Set[lanelet2.core.Lanelet],
    num_control_points: int = 50,
) -> Splines:
    """
    Extract centerline as spline from two adjacent lanelets using the left lanelet's right bound.

    Args:
        lanelet_map: The lanelet2 map containing the lanelets
        two_lanelets: Set containing exactly two lanelets
        num_control_points: Number of control points for B-spline interpolation

    Returns:
        Splines object representing the right bound of the left lanelet

    Raises:
        ValueError: If two_lanelets does not contain exactly 2 lanelets
    """
    if len(two_lanelets) != 2:
        raise ValueError(f"Expected exactly 2 lanelets, got {len(two_lanelets)}")

    # Sort the two lanelets from left to right
    sorted_lanelets = sort_adjacent_groups(lanelet_map, two_lanelets)

    # Get the left lanelet (first in sorted order)
    left_lanelet = sorted_lanelets[0]

    # Extract points from the right bound of the left lanelet
    right_bound = left_lanelet.rightBound

    if len(right_bound) < 2:
        raise ValueError("Left lanelet must have at least 2 points in its right bound")

    points = []
    for point in right_bound:
        points.append([point.x, point.y, point.z])

    points = np.array(points)

    # Create and return the B-spline
    return Splines(
        points,
        start_vel=_get_start_vel(left_lanelet),
        end_vel=_get_end_vel(left_lanelet),
        num_control_points=num_control_points,
    )
