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
    # Rotate 90 degrees clockwise: (x, y) -> (y, -x)
    perp_2d = np.array([segment[1], -segment[0]])

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
    # Rotate 90 degrees clockwise: (x, y) -> (y, -x)
    perp_2d = np.array([segment[1], -segment[0]])

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
    lanelet: lanelet2.core.Lanelet, num_control_points: int = 20
) -> Splines:
    """
    Extract centerline from a Lanelet using midpoints between left and right borders.

    Uses extract_border_from_spline to get left and right border splines, then samples
    points from both borders using normalized 0-1 Frenet coordinates and calculates
    midpoints to create the centerline spline.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_control_points: Number of control points for B-spline interpolation

    Returns:
        Splines object that can be evaluated using arc length
    """
    # Extract left and right border splines
    left_border_spline = extract_border_from_spline(lanelet, "left", num_control_points)
    right_border_spline = extract_border_from_spline(
        lanelet, "right", num_control_points
    )

    # Number of sample points for midpoint calculation
    num_samples = max(20, num_control_points * 2)

    # Sample points from both borders using normalized coordinates (0-1)
    centerline_points = []
    for i in range(num_samples):
        # Normalized coordinate from 0 to 1
        t_normalized = i / (num_samples - 1) if num_samples > 1 else 0.0

        # Convert to arc length for each border
        left_s = t_normalized * left_border_spline.total_length
        right_s = t_normalized * right_border_spline.total_length

        # Sample points from both borders
        left_point = left_border_spline.evaluate(left_s)
        right_point = right_border_spline.evaluate(right_s)

        # Calculate midpoint
        midpoint = [
            (left_point[0] + right_point[0]) / 2.0,
            (left_point[1] + right_point[1]) / 2.0,
            (left_point[2] + right_point[2]) / 2.0,
        ]
        centerline_points.append(midpoint)

    centerline_points = np.array(centerline_points)

    # Create B-spline with constrained fitting
    return Splines(centerline_points, num_control_points=num_control_points)


def extract_border_from_spline(
    lanelet: lanelet2.core.Lanelet, border: str, num_control_points: int = 20
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
    print(f"Creating {border} border spline with {num_control_points} control points")
    spline = Splines(points, num_control_points=num_control_points)
    print(
        f"{border} border spline created with total length {spline.total_length:.2f}m"
    )
    print(
        f"{spline.evaluate(0.0)-points[0]=}, {spline.evaluate(spline.total_length)-points[-1]=}"
    )
    return spline


def estimate_lanelet_width_as_spline(
    lanelet: lanelet2.core.Lanelet,
    num_samples: int = 20,
    num_control_points: int = 20,
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
    num_control_points: int = 20,
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
    return Splines(points, num_control_points=num_control_points)
