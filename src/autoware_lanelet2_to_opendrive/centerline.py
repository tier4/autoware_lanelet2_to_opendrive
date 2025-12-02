import numpy as np
import lanelet2
from splines import CatmullRom
from typing import Optional


def extract_centerline_as_spline(
    lanelet: lanelet2.core.Lanelet, alpha: float = 0.5
) -> CatmullRom:
    """
    Extract centerline from a Lanelet and return as a CatmullRom spline object.

    Args:
        lanelet: A Lanelet2 lanelet object
        alpha: Alpha parameter for Catmull-Rom spline (0=uniform, 0.5=centripetal, 1=chordal)

    Returns:
        CatmullRom spline object that can be evaluated at any parameter t in [0, 1]
    """
    centerline = lanelet.centerline

    if len(centerline) < 2:
        raise ValueError("Lanelet must have at least 2 points in its centerline")

    points = []
    for point in centerline:
        points.append([point.x, point.y, point.z])

    points = np.array(points)

    if len(points) < 4:
        raise ValueError(
            "Lanelet must have at least 4 points for Catmull-Rom spline. Use linear interpolation for fewer points."
        )

    # CatmullRom expects points as rows (N x D) where N is number of points, D is dimensions
    spline = CatmullRom(points, alpha=alpha)

    return spline


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
    centerline_spline = extract_centerline_as_spline(lanelet, alpha)

    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    left_bound_points = np.array([[p.x, p.y, p.z] for p in left_bound])
    right_bound_points = np.array([[p.x, p.y, p.z] for p in right_bound])

    t_values = np.linspace(0, 1, num_samples)
    total_widths = []

    for t in t_values:
        center_point = centerline_spline.evaluate(t).flatten()

        # Calculate tangent numerically
        dt = 0.001
        if t + dt <= 1.0:
            next_point = centerline_spline.evaluate(t + dt).flatten()
            tangent = (next_point - center_point) / dt
        else:
            prev_point = centerline_spline.evaluate(t - dt).flatten()
            tangent = (center_point - prev_point) / dt

        tangent = tangent / np.linalg.norm(tangent)

        normal = np.array([-tangent[1], tangent[0], 0])
        normal = normal / np.linalg.norm(normal)

        min_left_dist = float("inf")
        for i in range(len(left_bound_points) - 1):
            seg_start = left_bound_points[i]
            seg_end = left_bound_points[i + 1]

            dist = point_to_line_segment_distance(
                center_point, seg_start, seg_end, normal
            )
            if dist is not None and dist < min_left_dist:
                min_left_dist = dist

        min_right_dist = float("inf")
        for i in range(len(right_bound_points) - 1):
            seg_start = right_bound_points[i]
            seg_end = right_bound_points[i + 1]

            dist = point_to_line_segment_distance(
                center_point, seg_start, seg_end, -normal
            )
            if dist is not None and dist < min_right_dist:
                min_right_dist = dist

        left_width = min_left_dist if min_left_dist != float("inf") else 0.0
        right_width = min_right_dist if min_right_dist != float("inf") else 0.0
        total_widths.append(left_width + right_width)

    # Create 1D spline for total width values
    # CatmullRom expects points as rows: [[t0, width0], [t1, width1], ...]
    t_params = np.linspace(0, 1, num_samples)
    width_points = np.column_stack([t_params, total_widths])

    if num_samples >= 4:
        width_spline = CatmullRom(width_points, alpha=alpha)
    else:
        raise ValueError(
            "num_samples must be at least 4 for Catmull-Rom spline interpolation"
        )

    return width_spline


def point_to_line_segment_distance(
    point: np.ndarray,
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    direction: Optional[np.ndarray] = None,
) -> Optional[float]:
    """
    Calculate perpendicular distance from point to line segment in specified direction.

    Args:
        point: Point from which to measure distance
        seg_start: Start of line segment
        seg_end: End of line segment
        direction: Direction vector for perpendicular (if None, uses shortest distance)

    Returns:
        Distance if perpendicular intersects segment, None otherwise
    """
    seg_vec = seg_end - seg_start
    seg_length = np.linalg.norm(seg_vec)

    if seg_length < 1e-10:
        return np.linalg.norm(point[:2] - seg_start[:2])

    seg_unit = seg_vec / seg_length

    if direction is not None:
        direction = direction[:2] / np.linalg.norm(direction[:2])

        denom = np.dot(direction[:2], seg_unit[:2])
        if abs(denom) < 1e-10:
            return None

        to_start = seg_start[:2] - point[:2]
        t = np.dot(to_start, seg_unit[:2]) / denom

        if t < 0:
            return None

        intersection = point[:2] + t * direction[:2]

        projection = np.dot(intersection - seg_start[:2], seg_unit[:2])

        if 0 <= projection <= seg_length:
            return abs(t)
        else:
            return None
    else:
        point_vec = point[:2] - seg_start[:2]
        projection = np.dot(point_vec, seg_unit[:2])
        projection = np.clip(projection, 0, seg_length)

        closest = seg_start[:2] + projection * seg_unit[:2]
        return np.linalg.norm(point[:2] - closest)
