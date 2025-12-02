import numpy as np
import lanelet2
from splines import CatmullRom
from .geometry import (
    point_to_line_segment_distance,
    ArcLengthParameterizedCatmullRomSpline,
)


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
    lanelet: lanelet2.core.Lanelet, num_samples: int = 20, alpha: float = 0.5
) -> CatmullRom:
    """
    Estimate lanelet total width along its centerline using Frenet coordinates.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_samples: Number of sample points along the centerline
        alpha: Alpha parameter for Catmull-Rom spline

    Returns:
        CatmullRom spline object representing the total width (left + right distances)
    """

    # Get arc length parameterized centerline spline
    length_based_spline = extract_centerline_as_spline(lanelet, alpha)
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
        center_point = frenet_frame["position"]

        # Find closest distance to left boundary (use simple point-to-line distance)
        min_left_dist = float("inf")
        for i in range(len(left_bound_points) - 1):
            seg_start = left_bound_points[i]
            seg_end = left_bound_points[i + 1]

            dist = point_to_line_segment_distance(
                center_point, seg_start, seg_end, None
            )
            if dist is not None and dist < min_left_dist:
                min_left_dist = dist

        # Find closest distance to right boundary
        min_right_dist = float("inf")
        for i in range(len(right_bound_points) - 1):
            seg_start = right_bound_points[i]
            seg_end = right_bound_points[i + 1]

            dist = point_to_line_segment_distance(
                center_point, seg_start, seg_end, None
            )
            if dist is not None and dist < min_right_dist:
                min_right_dist = dist

        left_width = min_left_dist if min_left_dist != float("inf") else 0.0
        right_width = min_right_dist if min_right_dist != float("inf") else 0.0
        total_widths.append(left_width + right_width)

    # Create 1D spline for total width values
    # CatmullRom expects points as rows: [[length0, width0], [length1, width1], ...]
    width_points = np.column_stack([length_values, total_widths])

    return ArcLengthParameterizedCatmullRomSpline(width_points, alpha=alpha)
