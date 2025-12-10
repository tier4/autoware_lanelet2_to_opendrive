import numpy as np
from typing import Optional, List, Tuple, Dict
import lanelet2
import logging

logger = logging.getLogger(__name__)


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


def line_line_intersection_2d(
    p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray
) -> Optional[np.ndarray]:
    """
    Calculate the intersection point of two 2D lines defined by two points each.

    Args:
        p1, p2: Points defining the first line
        p3, p4: Points defining the second line

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


def align_point_to_reference_line(
    lanelet_map: lanelet2.core.LaneletMap,
    reference_points: Tuple[int, int],
    target_point_id: int,
) -> bool:
    """
    Align a target point to the intersection of its LineString with the reference LineStrings.

    The reference line is created from the segments of LineStrings containing the reference points.
    The target point is moved to where its LineString intersects with this reference line.

    Args:
        lanelet_map: Lanelet2 map containing the points
        reference_points: Tuple of two point IDs whose containing LineStrings define the reference line
        target_point_id: ID of the point to align

    Returns:
        True if alignment was successful, False otherwise
    """
    # Find all points in the map
    all_points = {}
    for lanelet in lanelet_map.laneletLayer:
        for point in lanelet.leftBound:
            all_points[point.id] = point
        for point in lanelet.rightBound:
            all_points[point.id] = point

    # Get target point
    target_point = all_points.get(target_point_id)
    if not target_point:
        logger.warning(f"Target point {target_point_id} not found in map")
        return False

    # Find LineStrings containing the reference points
    ref_linestrings_0 = find_linestring_containing_point(
        lanelet_map, reference_points[0]
    )
    ref_linestrings_1 = find_linestring_containing_point(
        lanelet_map, reference_points[1]
    )

    if not ref_linestrings_0:
        logger.warning(
            f"No LineString found containing reference point {reference_points[0]}"
        )
        return False
    if not ref_linestrings_1:
        logger.warning(
            f"No LineString found containing reference point {reference_points[1]}"
        )
        return False

    # Find LineStrings containing the target point
    target_linestrings = find_linestring_containing_point(lanelet_map, target_point_id)
    if not target_linestrings:
        logger.warning(f"No LineString found containing point {target_point_id}")
        return False

    intersection_found = False

    # Try all combinations of reference LineString segments and target LineString segments
    for ref_ls_0 in ref_linestrings_0:
        # Get segment for first reference point
        ref_segment_0 = get_linestring_segment_for_point(ref_ls_0, reference_points[0])
        if not ref_segment_0:
            continue

        for ref_ls_1 in ref_linestrings_1:
            # Get segment for second reference point
            ref_segment_1 = get_linestring_segment_for_point(
                ref_ls_1, reference_points[1]
            )
            if not ref_segment_1:
                continue

            # Convert reference segments to numpy arrays
            ref_seg0_p1 = np.array(
                [ref_segment_0[0].x, ref_segment_0[0].y, ref_segment_0[0].z]
            )
            ref_seg0_p2 = np.array(
                [ref_segment_0[1].x, ref_segment_0[1].y, ref_segment_0[1].z]
            )
            ref_seg1_p1 = np.array(
                [ref_segment_1[0].x, ref_segment_1[0].y, ref_segment_1[0].z]
            )
            ref_seg1_p2 = np.array(
                [ref_segment_1[1].x, ref_segment_1[1].y, ref_segment_1[1].z]
            )

            # Process each LineString containing the target point
            for target_ls in target_linestrings:
                # Get the segment containing the target point
                target_segment = get_linestring_segment_for_point(
                    target_ls, target_point_id
                )
                if not target_segment:
                    continue

                # Convert target segment points to numpy arrays
                target_seg_p1 = np.array(
                    [target_segment[0].x, target_segment[0].y, target_segment[0].z]
                )
                target_seg_p2 = np.array(
                    [target_segment[1].x, target_segment[1].y, target_segment[1].z]
                )

                # Try intersection with first reference segment
                intersection = line_line_intersection_2d(
                    ref_seg0_p1, ref_seg0_p2, target_seg_p1, target_seg_p2
                )

                # If no intersection with first segment, try second reference segment
                if intersection is None:
                    intersection = line_line_intersection_2d(
                        ref_seg1_p1, ref_seg1_p2, target_seg_p1, target_seg_p2
                    )

                if intersection is not None:
                    # Log before and after coordinates
                    logger.info(
                        f"Moving point {target_point_id} from ({target_point.x:.3f}, {target_point.y:.3f}) "
                        f"to ({intersection[0]:.3f}, {intersection[1]:.3f})"
                    )

                    # Update the target point position
                    target_point.x = intersection[0]
                    target_point.y = intersection[1]
                    if len(intersection) > 2:
                        target_point.z = intersection[2]

                    # Verify the update
                    logger.info(
                        f"Point {target_point_id} now at ({target_point.x:.3f}, {target_point.y:.3f})"
                    )

                    intersection_found = True
                    return True  # Exit once we find a valid intersection

    if not intersection_found:
        logger.warning(f"No intersection found for point {target_point_id}")

    return intersection_found


def align_points_to_reference(
    lanelet_map: lanelet2.core.LaneletMap,
    reference_points: Tuple[int, int],
    target_points: List[int],
) -> Dict[int, bool]:
    """
    Align multiple target points to their intersections with a reference line.

    Args:
        lanelet_map: Lanelet2 map containing the points
        reference_points: Tuple of two point IDs defining the reference line
        target_points: List of point IDs to align

    Returns:
        Dictionary mapping point IDs to success status
    """
    results = {}

    logger.info(
        f"Aligning {len(target_points)} points to reference line defined by points {reference_points}"
    )

    for point_id in target_points:
        success = align_point_to_reference_line(lanelet_map, reference_points, point_id)
        results[point_id] = success

        if not success:
            logger.warning(f"Failed to align point {point_id}")

    # Log summary
    successful = sum(1 for s in results.values() if s)
    logger.info(f"Successfully aligned {successful}/{len(target_points)} points")

    return results


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
