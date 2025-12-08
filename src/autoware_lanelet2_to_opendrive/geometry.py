import numpy as np
from typing import Optional


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

    if seg_length < 1e-10:
        return np.linalg.norm(point[:2] - seg_start[:2])

    seg_unit = seg_vec / seg_length

    if direction is not None:
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

            # Check if intersection is on the line segment (0 <= s <= seg_length)
            # and ray goes in positive direction (t >= 0)
            if 0 <= s <= seg_length and t >= 0:
                return t  # Distance along the ray
            else:
                return None
        except np.linalg.LinAlgError:
            # Matrix is singular (parallel lines)
            return None
    else:
        point_vec = point[:2] - seg_start[:2]
        projection = np.dot(point_vec, seg_unit)
        projection = np.clip(projection, 0, seg_length)

        closest = seg_start[:2] + projection * seg_unit
        return np.linalg.norm(point[:2] - closest)
