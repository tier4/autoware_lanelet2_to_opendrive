import numpy as np
from typing import Optional, List, Tuple, Dict, Union
import lanelet2
import logging

from .config import DEFAULT_CONFIG
from .types import Point2D, Point3D

logger = logging.getLogger(__name__)


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


def ensure_point3d(p: Union[Point3D, np.ndarray, List[float]]) -> Point3D:
    """Convert any 3D point representation to Point3D.

    This helper function provides backward compatibility with legacy code
    using numpy arrays or lists, while also accepting the new Point3D type.

    Args:
        p: Point as Point3D, numpy array, or list (must have 3 elements)

    Returns:
        Point3D instance

    Example:
        ```python
        # Works with all representations
        p1 = ensure_point3d([1.0, 2.0, 3.0])
        p2 = ensure_point3d(np.array([1.0, 2.0, 3.0]))
        p3 = ensure_point3d(Point3D(1.0, 2.0, 3.0))
        ```
    """
    if isinstance(p, Point3D):
        return p
    # Handle numpy array or list
    return Point3D.from_array(p)


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


def find_linestring_containing_point(
    lanelet_map: lanelet2.core.LaneletMap, point_id: int
) -> List[lanelet2.core.LineString3d]:
    """
    Find all LineString3d objects that contain a point with the given ID.

    Args:
        lanelet_map: Lanelet2 map to search
        point_id: ID of the point to find

    Returns:
        List of LineString3d objects containing the point
    """
    linestrings = []
    seen_ids = set()  # Track already found linestrings to avoid duplicates

    # Search through all lanelets
    for lanelet in lanelet_map.laneletLayer:
        # Check left boundary
        if lanelet.leftBound.id not in seen_ids:
            for point in lanelet.leftBound:
                if point.id == point_id:
                    linestrings.append(lanelet.leftBound)
                    seen_ids.add(lanelet.leftBound.id)
                    break

        # Check right boundary
        if lanelet.rightBound.id not in seen_ids:
            for point in lanelet.rightBound:
                if point.id == point_id:
                    linestrings.append(lanelet.rightBound)
                    seen_ids.add(lanelet.rightBound.id)
                    break

    # Search through linestring layer if it exists
    if hasattr(lanelet_map, "lineStringLayer"):
        for linestring in lanelet_map.lineStringLayer:
            if linestring.id not in seen_ids:
                for point in linestring:
                    if point.id == point_id:
                        linestrings.append(linestring)
                        seen_ids.add(linestring.id)
                        break

    return linestrings


def get_linestring_segment_for_point(
    linestring: lanelet2.core.LineString3d, point_id: int
) -> Optional[Tuple[lanelet2.core.Point3d, lanelet2.core.Point3d]]:
    """
    Get the line segment (prev_point, next_point) for a point in a linestring.

    Args:
        linestring: LineString3d containing the point
        point_id: ID of the point

    Returns:
        Tuple of (previous_point, next_point) or None if point not found or at boundary
    """
    points = list(linestring)

    for i, point in enumerate(points):
        if point.id == point_id:
            # Get previous and next points if available
            prev_point = points[i - 1] if i > 0 else None
            next_point = points[i + 1] if i < len(points) - 1 else None

            # If point is at the beginning, use next two points
            if prev_point is None and next_point and i + 2 < len(points):
                return (next_point, points[i + 2])

            # If point is at the end, use previous two points
            if next_point is None and prev_point and i - 2 >= 0:
                return (points[i - 2], prev_point)

            # Normal case: point is in the middle
            if prev_point and next_point:
                return (prev_point, next_point)

    return None


def delete_points_from_map(
    lanelet_map: lanelet2.core.LaneletMap, point_ids: List[int]
) -> Dict[int, bool]:
    """
    Delete specified points from all LineStrings in the map.

    Args:
        lanelet_map: Lanelet2 map containing the points
        point_ids: List of point IDs to delete

    Returns:
        Dictionary mapping point IDs to deletion success status
    """
    results = {}
    point_ids_set = set(point_ids)

    logger.info(f"Deleting {len(point_ids)} points from map")

    # Process all lanelets
    for lanelet in lanelet_map.laneletLayer:
        # Process left boundary
        left_points = list(lanelet.leftBound)
        modified_left = False
        new_left_points = []

        for point in left_points:
            if point.id not in point_ids_set:
                new_left_points.append(point)
            else:
                modified_left = True
                results[point.id] = True
                logger.debug(
                    f"  Removing point {point.id} from lanelet {lanelet.id} left bound"
                )

        # Update left boundary if points were removed
        if modified_left and len(new_left_points) >= 2:
            # Create new LineString with remaining points
            new_left_bound = lanelet2.core.LineString3d(
                lanelet.leftBound.id, new_left_points
            )

            # Replace the lanelet's left bound
            lanelet.leftBound = new_left_bound
        elif modified_left and len(new_left_points) < 2:
            logger.warning(
                f"  Cannot remove points from lanelet {lanelet.id} left bound - would result in less than 2 points"
            )
            # Mark points as not deleted if removal would break the linestring
            for point in left_points:
                if point.id in point_ids_set:
                    results[point.id] = False

        # Process right boundary
        right_points = list(lanelet.rightBound)
        modified_right = False
        new_right_points = []

        for point in right_points:
            if point.id not in point_ids_set:
                new_right_points.append(point)
            else:
                modified_right = True
                results[point.id] = True
                logger.debug(
                    f"  Removing point {point.id} from lanelet {lanelet.id} right bound"
                )

        # Update right boundary if points were removed
        if modified_right and len(new_right_points) >= 2:
            # Create new LineString with remaining points
            new_right_bound = lanelet2.core.LineString3d(
                lanelet.rightBound.id, new_right_points
            )

            # Replace the lanelet's right bound
            lanelet.rightBound = new_right_bound
        elif modified_right and len(new_right_points) < 2:
            logger.warning(
                f"  Cannot remove points from lanelet {lanelet.id} right bound - would result in less than 2 points"
            )
            # Mark points as not deleted if removal would break the linestring
            for point in right_points:
                if point.id in point_ids_set:
                    results[point.id] = False

    # Note: LineString layer handling is commented out as LaneletMap doesn't
    # provide a simple way to modify standalone linestrings.
    # Points are primarily removed from lanelet boundaries above.

    # Check for points that weren't found
    for point_id in point_ids:
        if point_id not in results:
            results[point_id] = False
            logger.warning(f"  Point {point_id} not found in any LineString")

    # Log summary
    successful = sum(1 for s in results.values() if s)
    logger.info(f"Successfully deleted {successful}/{len(point_ids)} points")

    return results


def move_point_in_map(
    lanelet_map: lanelet2.core.LaneletMap,
    point_id: int,
    new_x: float,
    new_y: float,
    new_z: Optional[float] = None,
) -> bool:
    """
    Move a point to new coordinates in the map.

    Args:
        lanelet_map: Lanelet2 map containing the point
        point_id: ID of the point to move
        new_x: New X coordinate
        new_y: New Y coordinate
        new_z: New Z coordinate (if None, keep original)

    Returns:
        True if point was found and moved, False otherwise
    """
    # Find all points in the map
    all_points = {}
    for lanelet in lanelet_map.laneletLayer:
        for point in lanelet.leftBound:
            all_points[point.id] = point
        for point in lanelet.rightBound:
            all_points[point.id] = point

    # Get target point
    target_point = all_points.get(point_id)
    if not target_point:
        logger.warning(f"Point {point_id} not found in map")
        return False

    # Log before and after coordinates
    final_z = new_z if new_z is not None else target_point.z

    logger.info(
        f"Moving point {point_id} from ({target_point.x:.3f}, {target_point.y:.3f}, {target_point.z:.3f}) "
        f"to ({new_x:.3f}, {new_y:.3f}, {final_z:.3f})"
    )

    # Create a new Point3d with the updated coordinates
    new_point = lanelet2.core.Point3d(
        point_id,
        new_x,
        new_y,
        final_z,
    )

    # Copy attributes from original point and update local coordinates
    for key, value in target_point.attributes.items():
        new_point.attributes[key] = value

    # Update local coordinate attributes
    new_point.attributes["local_x"] = f"{new_x:.4f}"
    new_point.attributes["local_y"] = f"{new_y:.4f}"
    if new_z is not None:
        new_point.attributes["ele"] = f"{new_z:.12f}"

    # Update all LineStrings containing this point
    _update_point_in_map(lanelet_map, point_id, new_point)

    # Verify the update
    logger.info(
        f"Point {point_id} updated to ({new_x:.3f}, {new_y:.3f}, {final_z:.3f})"
    )

    return True


def move_points_in_map(
    lanelet_map: lanelet2.core.LaneletMap,
    point_moves: List[tuple[int, float, float, Optional[float]]],
) -> Dict[int, bool]:
    """
    Move multiple points in the map.

    Args:
        lanelet_map: Lanelet2 map containing the points
        point_moves: List of tuples (point_id, new_x, new_y, new_z)

    Returns:
        Dictionary mapping point IDs to move success status
    """
    results = {}

    logger.info(f"Moving {len(point_moves)} points in map")

    for point_id, new_x, new_y, new_z in point_moves:
        success = move_point_in_map(lanelet_map, point_id, new_x, new_y, new_z)
        results[point_id] = success

        if not success:
            logger.warning(f"Failed to move point {point_id}")

    # Log summary
    successful = sum(1 for s in results.values() if s)
    logger.info(f"Successfully moved {successful}/{len(point_moves)} points")

    return results


def _update_point_in_map(
    lanelet_map: lanelet2.core.LaneletMap,
    point_id: int,
    new_point: lanelet2.core.Point3d,
) -> None:
    """
    Update a point in the lanelet map by recreating all LineStrings and Lanelets that contain it.

    Args:
        lanelet_map: The lanelet map to update
        point_id: ID of the point to update
        new_point: New point with updated coordinates
    """
    # Find all lanelets that contain this point
    lanelets_to_update = []

    for lanelet in lanelet_map.laneletLayer:
        needs_update = False
        new_left_points = []
        new_right_points = []

        # Check left boundary
        for point in lanelet.leftBound:
            if point.id == point_id:
                new_left_points.append(new_point)
                needs_update = True
            else:
                new_left_points.append(point)

        # Check right boundary
        for point in lanelet.rightBound:
            if point.id == point_id:
                new_right_points.append(new_point)
                needs_update = True
            else:
                new_right_points.append(point)

        if needs_update:
            # Create new LineStrings with updated points
            new_left_bound = lanelet2.core.LineString3d(
                lanelet.leftBound.id, new_left_points
            )
            new_right_bound = lanelet2.core.LineString3d(
                lanelet.rightBound.id, new_right_points
            )

            # Create new lanelet with updated boundaries
            new_lanelet = lanelet2.core.Lanelet(
                lanelet.id, new_left_bound, new_right_bound
            )

            # Copy attributes
            for key, value in lanelet.attributes.items():
                new_lanelet.attributes[key] = value

            # Copy regulatory elements
            for reg_elem in lanelet.regulatoryElements:
                new_lanelet.addRegulatoryElement(reg_elem)

            lanelets_to_update.append((lanelet, new_lanelet))

    # Update the lanelets in place by replacing their boundaries
    for old_lanelet, new_lanelet in lanelets_to_update:
        old_lanelet.leftBound = new_lanelet.leftBound
        old_lanelet.rightBound = new_lanelet.rightBound

        logger.debug(f"Updated lanelet {old_lanelet.id} with new point coordinates")
