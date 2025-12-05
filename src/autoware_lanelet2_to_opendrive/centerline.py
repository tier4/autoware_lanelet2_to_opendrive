import numpy as np
import lanelet2
from typing import Set
from .geometry import (
    point_to_line_segment_distance,
    ArcLengthParameterizedCatmullRomSpline,
)
from .util import sort_adjacent_groups


class AsymmetryLaneletException(Exception):
    """Exception raised when a lanelet has asymmetric left and right widths."""

    pass


def extract_centerline_as_spline(
    lanelet: lanelet2.core.Lanelet, alpha: float = 0.5
) -> ArcLengthParameterizedCatmullRomSpline:
    """
    Extract centerline from a Lanelet and return as arc length parameterized spline.

    Args:
        lanelet: A Lanelet2 lanelet object
        alpha: Alpha parameter for Catmull-Rom spline (0=uniform, 0.5=centripetal, 1=chordal)

    Returns:
        ArcLengthParameterizer object that can be evaluated using arc length
    """
    centerline = lanelet.centerline

    if len(centerline) < 2:
        raise ValueError("Lanelet must have at least 2 points in its centerline")

    points = []
    for point in centerline:
        points.append([point.x, point.y, point.z])

    points = np.array(points)

    # Use the new function from geometry.py
    return ArcLengthParameterizedCatmullRomSpline(points, alpha)


def estimate_lanelet_width_as_spline(
    lanelet: lanelet2.core.Lanelet,
    num_samples: int = 20,
    alpha: float = 0.5,
    reference: str = "center_line",
) -> ArcLengthParameterizedCatmullRomSpline:
    """
    Estimate lanelet total width along its centerline or left boundary using Frenet coordinates.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_samples: Number of sample points along the reference line
        alpha: Alpha parameter for Catmull-Rom spline
        reference: Reference line for width calculation - "center_line" or "left_bound"

    Returns:
        CatmullRom spline object representing the total width (left + right distances)
    """
    if reference not in ["center_line", "left_bound"]:
        raise ValueError(
            f"Invalid reference: {reference}. Must be 'center_line' or 'left_bound'"
        )

    # Get arc length parameterized reference spline based on mode
    if reference == "center_line":
        length_based_spline = extract_centerline_as_spline(lanelet, alpha)
    else:  # left_bound
        length_based_spline = extract_left_boundary_as_spline(lanelet, alpha)

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
        frenet_frame = length_based_spline.evaluate(length, frenet=True)
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
    # CatmullRom expects points as rows: [[length0, width0], [length1, width1], ...]
    width_points = np.column_stack([length_values, total_widths])

    return ArcLengthParameterizedCatmullRomSpline(width_points, alpha=alpha)


def extract_centerline_as_spline_from_two_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    two_lanelets: Set[lanelet2.core.Lanelet],
    alpha: float = 0.5,
) -> ArcLengthParameterizedCatmullRomSpline:
    """
    Extract centerline as spline from two adjacent lanelets using the left lanelet's right bound.

    Args:
        lanelet_map: The lanelet2 map containing the lanelets
        two_lanelets: Set containing exactly two lanelets
        alpha: Alpha parameter for Catmull-Rom spline (0=uniform, 0.5=centripetal, 1=chordal)

    Returns:
        ArcLengthParameterizedCatmullRomSpline representing the right bound of the left lanelet

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

    # Create and return the spline
    return ArcLengthParameterizedCatmullRomSpline(points, alpha)


def extract_left_boundary_as_spline(
    lanelet: lanelet2.core.Lanelet, alpha: float = 0.5
) -> ArcLengthParameterizedCatmullRomSpline:
    """
    Extract left boundary from a Lanelet and return as arc length parameterized spline.

    Args:
        lanelet: A Lanelet2 lanelet object
        alpha: Alpha parameter for Catmull-Rom spline (0=uniform, 0.5=centripetal, 1=chordal)

    Returns:
        ArcLengthParameterizedCatmullRomSpline representing the left boundary
    """
    left_bound = lanelet.leftBound

    if len(left_bound) < 2:
        raise ValueError("Lanelet must have at least 2 points in its left bound")

    points = []
    for point in left_bound:
        points.append([point.x, point.y, point.z])

    points = np.array(points)

    return ArcLengthParameterizedCatmullRomSpline(points, alpha)
