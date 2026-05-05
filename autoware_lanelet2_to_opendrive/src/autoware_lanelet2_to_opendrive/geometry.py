import lanelet2
import numpy as np
from typing import Optional, List, Tuple, Union

from .config import DEFAULT_CONFIG
from .types import Point2D, Point3D


def ensure_point2d(p: Union[Point2D, Point3D, np.ndarray, List[float]]) -> Point2D:
    """Convert any point representation to Point2D.

    This helper function provides backward compatibility with legacy code
    using numpy arrays or lists, while also accepting the new Point types.

    Args:
        p: Point as Point2D, Point3D, numpy array, or list

    Returns:
        Point2D instance

    Example:
        ```python
        # Works with all representations
        p1 = ensure_point2d([1.0, 2.0])
        p2 = ensure_point2d(np.array([1.0, 2.0]))
        p3 = ensure_point2d(Point2D(1.0, 2.0))
        p4 = ensure_point2d(Point3D(1.0, 2.0, 3.0))  # Drops z
        ```
    """
    if isinstance(p, Point2D):
        return p
    if isinstance(p, Point3D):
        return p.to_2d()
    # Handle numpy array or list
    return Point2D.from_array(p)


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
    # Use only 2D coordinates for all calculations
    seg_vec = seg_end[:2] - seg_start[:2]
    seg_length = np.linalg.norm(seg_vec)

    # Guard: Handle degenerate segment
    if seg_length < DEFAULT_CONFIG.geometry.epsilon:
        return np.linalg.norm(point[:2] - seg_start[:2])

    seg_unit = seg_vec / seg_length

    # Guard: If no direction specified, use shortest distance
    if direction is None:
        return _calculate_shortest_distance_to_segment(
            point, seg_start, seg_unit, seg_length
        )

    # Calculate distance in specified direction
    return _calculate_distance_in_direction(
        point, seg_start, seg_unit, seg_length, direction
    )


def _calculate_distance_in_direction(
    point: np.ndarray,
    seg_start: np.ndarray,
    seg_unit: np.ndarray,
    seg_length: float,
    direction: np.ndarray,
) -> Optional[float]:
    """
    Calculate distance from point to segment along a specified direction.

    Args:
        point: Point from which to measure distance
        seg_start: Start of line segment
        seg_unit: Unit direction vector of segment
        seg_length: Length of segment
        direction: Direction vector for measurement

    Returns:
        Distance along direction if ray intersects segment, None otherwise
    """
    direction_2d = direction[:2] / np.linalg.norm(direction[:2])

    # Find intersection of ray from point in given direction with infinite line
    # Line equation: seg_start + s * seg_unit
    # Ray equation: point + t * direction
    # Solve: seg_start + s * seg_unit = point + t * direction

    # Create 2x2 matrix: [seg_unit, -direction]
    # Solve for s and t
    A = np.column_stack([seg_unit, -direction_2d])
    b = point[:2] - seg_start[:2]

    try:
        # Solve linear system
        solution = np.linalg.solve(A, b)
        s, t = solution

        # Guard: Check if intersection is valid
        # Must be on line segment (0 <= s <= seg_length)
        # and ray must go in positive direction (t >= 0)
        if not (0 <= s <= seg_length and t >= 0):
            return None

        return t  # Distance along the ray

    except np.linalg.LinAlgError:
        # Matrix is singular (parallel lines)
        return None


def _calculate_shortest_distance_to_segment(
    point: np.ndarray,
    seg_start: np.ndarray,
    seg_unit: np.ndarray,
    seg_length: float,
) -> float:
    """
    Calculate shortest distance from point to line segment.

    Args:
        point: Point from which to measure distance
        seg_start: Start of line segment
        seg_unit: Unit direction vector of segment
        seg_length: Length of segment

    Returns:
        Shortest distance from point to segment
    """
    point_vec = point[:2] - seg_start[:2]
    projection = np.dot(point_vec, seg_unit)
    projection = np.clip(projection, 0, seg_length)

    closest = seg_start[:2] + projection * seg_unit
    return np.linalg.norm(point[:2] - closest)


def line_line_intersection_2d(
    p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray
) -> Optional[np.ndarray]:
    """
    Calculate the intersection point of two 2D lines defined by two points each.

    **Legacy function**: This function accepts numpy arrays for backward compatibility.
    For new code, consider using `line_line_intersection_typed` with Point2D/Point3D.

    Args:
        p1, p2: Points defining the first line (numpy array or list)
        p3, p4: Points defining the second line (numpy array or list)

    Returns:
        Intersection point as numpy array, or None if lines are parallel
    """
    # Convert to 2D if necessary
    p1_2d = p1[:2] if len(p1) > 2 else p1
    p2_2d = p2[:2] if len(p2) > 2 else p2
    p3_2d = p3[:2] if len(p3) > 2 else p3
    p4_2d = p4[:2] if len(p4) > 2 else p4

    # Line 1: p1 + t * (p2 - p1)
    # Line 2: p3 + s * (p4 - p3)
    d1 = p2_2d - p1_2d
    d2 = p4_2d - p3_2d

    # Solve: p1 + t * d1 = p3 + s * d2
    # Rearrange: t * d1 - s * d2 = p3 - p1

    # Create matrix [d1, -d2] and solve for [t, s]
    A = np.column_stack([d1, -d2])
    b = p3_2d - p1_2d

    try:
        solution = np.linalg.solve(A, b)
        t = solution[0]

        # Calculate intersection point
        intersection = p1_2d + t * d1

        # Add z-coordinate if original points were 3D
        if len(p1) > 2:
            # Interpolate z from the reference line
            z = p1[2] + t * (p2[2] - p1[2])
            intersection = np.append(intersection, z)

        return intersection

    except np.linalg.LinAlgError:
        # Lines are parallel or coincident
        return None


def line_line_intersection_typed(
    p1: Union[Point2D, Point3D],
    p2: Union[Point2D, Point3D],
    p3: Union[Point2D, Point3D],
    p4: Union[Point2D, Point3D],
) -> Optional[Union[Point2D, Point3D]]:
    """
    Calculate the intersection point of two lines with type-safe Point classes.

    This is the type-safe version of `line_line_intersection_2d` that uses
    Point2D/Point3D instead of numpy arrays. Dimension errors are caught at
    type-check time rather than runtime.

    Args:
        p1, p2: Points defining the first line
        p3, p4: Points defining the second line

    Returns:
        Intersection point (same type as input points), or None if lines are parallel

    Example:
        ```python
        # Type-safe 2D intersection
        p1 = Point2D(0.0, 0.0)
        p2 = Point2D(1.0, 1.0)
        p3 = Point2D(0.0, 1.0)
        p4 = Point2D(1.0, 0.0)
        intersection = line_line_intersection_typed(p1, p2, p3, p4)

        # Type-safe 3D intersection (projects to 2D, returns 3D with interpolated z)
        p1_3d = Point3D(0.0, 0.0, 0.0)
        p2_3d = Point3D(1.0, 1.0, 1.0)
        p3_3d = Point3D(0.0, 1.0, 0.5)
        p4_3d = Point3D(1.0, 0.0, 0.5)
        intersection_3d = line_line_intersection_typed(p1_3d, p2_3d, p3_3d, p4_3d)
        ```
    """
    # Determine if we're working with 3D points
    is_3d = isinstance(p1, Point3D)

    # Convert to 2D for intersection calculation
    p1_2d = ensure_point2d(p1)
    p2_2d = ensure_point2d(p2)
    p3_2d = ensure_point2d(p3)
    p4_2d = ensure_point2d(p4)

    # Line 1: p1 + t * (p2 - p1)
    # Line 2: p3 + s * (p4 - p3)
    d1 = p2_2d.to_array() - p1_2d.to_array()
    d2 = p4_2d.to_array() - p3_2d.to_array()

    # Solve: p1 + t * d1 = p3 + s * d2
    # Rearrange: t * d1 - s * d2 = p3 - p1
    A = np.column_stack([d1, -d2])
    b = p3_2d.to_array() - p1_2d.to_array()

    try:
        solution = np.linalg.solve(A, b)
        t = solution[0]

        # Calculate intersection point in 2D
        intersection_arr = p1_2d.to_array() + t * d1
        intersection_2d = Point2D.from_array(intersection_arr)

        # If original points were 3D, interpolate z and return Point3D
        if is_3d:
            assert isinstance(p1, Point3D) and isinstance(p2, Point3D)
            z = p1.z + t * (p2.z - p1.z)
            return Point3D(intersection_2d.x, intersection_2d.y, z)

        return intersection_2d

    except np.linalg.LinAlgError:
        # Lines are parallel or coincident
        return None


def compute_point_layer_bounds(
    lanelet_map: "lanelet2.core.LaneletMap",
) -> Tuple[float, float, float, float]:
    """Return (min_x, min_y, max_x, max_y) over all Point3d in pointLayer.

    The values are the axis-aligned bounding box of every projected point
    already loaded into the lanelet2 map. They map onto OpenDRIVE
    ``<header>`` attributes as: ``west=min_x``, ``south=min_y``,
    ``east=max_x``, ``north=max_y``.

    Parameters
    ----------
    lanelet_map : lanelet2.core.LaneletMap
        Map whose ``pointLayer`` will be scanned.

    Returns
    -------
    tuple of (float, float, float, float)
        ``(min_x, min_y, max_x, max_y)`` in projected (metric) coordinates.

    Raises
    ------
    ValueError
        If ``lanelet_map.pointLayer`` is empty. Falling back to zeros would
        silently reproduce the bug fixed by issue #465.
    """
    points = lanelet_map.pointLayer
    if len(points) == 0:
        raise ValueError(
            "Cannot compute header bounds: lanelet_map.pointLayer is empty."
        )

    iterator = iter(points)
    first = next(iterator)
    min_x = max_x = first.x
    min_y = max_y = first.y
    for p in iterator:
        if p.x < min_x:
            min_x = p.x
        if p.x > max_x:
            max_x = p.x
        if p.y < min_y:
            min_y = p.y
        if p.y > max_y:
            max_y = p.y
    return min_x, min_y, max_x, max_y
