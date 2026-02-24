"""Functions for working with Lanelet2 lanelet objects."""

from typing import Optional, List, Union, Dict, Set
import lanelet2
import logging
import numpy as np

logger = logging.getLogger(__name__)


def merge_lanelet(
    first_lanelet: lanelet2.core.Lanelet,
    second_lanelet: lanelet2.core.Lanelet,
    base_id: Optional[int] = None,
) -> lanelet2.core.Lanelet:
    """
    Merge two Lanelets by connecting their Left/Right Polygons to create one Lanelet.

    This function takes two consecutive lanelets and merges them into a single lanelet
    by connecting their boundaries. The resulting lanelet will have:
    - Left boundary: connected left boundaries of both lanelets
    - Right boundary: connected right boundaries of both lanelets

    Args:
        first_lanelet: First lanelet (predecessor)
        second_lanelet: Second lanelet (successor)
        base_id: Optional base ID for creating new IDs. If None, uses timestamp-based IDs.

    Returns:
        New merged Lanelet object

    Raises:
        ValueError: If the lanelets cannot be merged (incompatible boundaries)
    """
    # Get boundary linestrings from both lanelets
    left_bound1 = first_lanelet.leftBound
    right_bound1 = first_lanelet.rightBound
    left_bound2 = second_lanelet.leftBound
    right_bound2 = second_lanelet.rightBound

    # Validate that boundaries exist and have points
    if len(left_bound1) < 2 or len(right_bound1) < 2:
        raise ValueError(
            "First lanelet must have valid left and right boundaries with at least 2 points"
        )
    if len(left_bound2) < 2 or len(right_bound2) < 2:
        raise ValueError(
            "Second lanelet must have valid left and right boundaries with at least 2 points"
        )

    # Generate IDs if not provided
    if base_id is None:
        import time

        base_id = int(time.time() * 1000) % 1000000  # Use timestamp for unique IDs

    # Connect left boundaries
    merged_left_points = []
    # Add all points from first lanelet's left boundary
    for point in left_bound1:
        merged_left_points.append(point)

    # Add points from second lanelet's left boundary (skip first point to avoid duplication)
    for i, point in enumerate(left_bound2):
        if i > 0:  # Skip first point to avoid duplication at connection
            merged_left_points.append(point)

    # Connect right boundaries
    merged_right_points = []
    # Add all points from first lanelet's right boundary
    for point in right_bound1:
        merged_right_points.append(point)

    # Add points from second lanelet's right boundary (skip first point to avoid duplication)
    for i, point in enumerate(right_bound2):
        if i > 0:  # Skip first point to avoid duplication at connection
            merged_right_points.append(point)

    # Create new linestrings for the merged boundaries
    merged_left_bound = lanelet2.core.LineString3d(base_id + 1, merged_left_points)

    merged_right_bound = lanelet2.core.LineString3d(base_id + 2, merged_right_points)

    # Create the merged lanelet
    merged_lanelet = lanelet2.core.Lanelet(
        base_id + 3, merged_left_bound, merged_right_bound
    )

    # Copy attributes from the first lanelet (can be customized as needed)
    for key, value in first_lanelet.attributes.items():
        merged_lanelet.attributes[key] = value

    return merged_lanelet


def validate_lanelet_continuity(
    first_lanelet: lanelet2.core.Lanelet,
    second_lanelet: lanelet2.core.Lanelet,
    tolerance: float = 1e-3,
) -> bool:
    """
    Validate that two lanelets can be merged by checking boundary continuity.

    Args:
        first_lanelet: First lanelet (predecessor)
        second_lanelet: Second lanelet (successor)
        tolerance: Distance tolerance for boundary endpoint matching

    Returns:
        True if lanelets can be merged, False otherwise
    """
    # Get end points of first lanelet
    left1_end = first_lanelet.leftBound[-1]
    right1_end = first_lanelet.rightBound[-1]

    # Get start points of second lanelet
    left2_start = second_lanelet.leftBound[0]
    right2_start = second_lanelet.rightBound[0]

    # Calculate distances between corresponding endpoints
    left_distance = np.sqrt(
        (left1_end.x - left2_start.x) ** 2
        + (left1_end.y - left2_start.y) ** 2
        + (left1_end.z - left2_start.z) ** 2
    )

    right_distance = np.sqrt(
        (right1_end.x - right2_start.x) ** 2
        + (right1_end.y - right2_start.y) ** 2
        + (right1_end.z - right2_start.z) ** 2
    )

    # Check if both boundaries are continuous within tolerance
    return left_distance <= tolerance and right_distance <= tolerance


def merge_lanelets(
    lanelets: List[lanelet2.core.Lanelet],
    base_id: Optional[int] = None,
    validate: bool = True,
    tolerance: float = 1e-3,
) -> lanelet2.core.Lanelet:
    """
    Merge multiple consecutive Lanelets into a single Lanelet.

    This function iteratively merges a list of lanelets by calling merge_lanelet
    on consecutive pairs. The lanelets should be in sequential order.

    Args:
        lanelets: List of lanelets to merge (must be in sequential order)
        base_id: Optional base ID for creating new IDs. If None, uses timestamp-based IDs.
        validate: If True, validates continuity between consecutive lanelets before merging
        tolerance: Distance tolerance for boundary continuity validation (only used if validate=True)

    Returns:
        Single merged Lanelet object combining all input lanelets

    Raises:
        ValueError: If less than 2 lanelets provided or if validation fails
    """
    if not lanelets:
        raise ValueError("No lanelets provided to merge")

    if len(lanelets) == 1:
        raise ValueError("At least 2 lanelets required for merging")

    if len(lanelets) == 2:
        # Base case: merge just two lanelets
        if validate and not validate_lanelet_continuity(
            lanelets[0], lanelets[1], tolerance
        ):
            raise ValueError(
                f"Lanelets {lanelets[0].id} and {lanelets[1].id} are not continuous "
                f"(distance exceeds tolerance {tolerance})"
            )
        return merge_lanelet(lanelets[0], lanelets[1], base_id)

    # For more than 2 lanelets, merge iteratively
    # Validate all pairs first if requested
    if validate:
        for i in range(len(lanelets) - 1):
            if not validate_lanelet_continuity(lanelets[i], lanelets[i + 1], tolerance):
                raise ValueError(
                    f"Lanelets {lanelets[i].id} and {lanelets[i+1].id} are not continuous "
                    f"(distance exceeds tolerance {tolerance})"
                )

    # Generate base ID if not provided
    if base_id is None:
        import time

        base_id = int(time.time() * 1000) % 1000000

    # Start with merging the first two lanelets
    merged = merge_lanelet(lanelets[0], lanelets[1], base_id)

    # Iteratively merge with remaining lanelets
    for i in range(2, len(lanelets)):
        # Generate new ID for each merge operation to avoid conflicts
        next_base_id = base_id + (i * 10)  # Offset by 10 to avoid ID collisions
        merged = merge_lanelet(merged, lanelets[i], next_base_id)

    return merged


def merge_lanelets_from_ids(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelet_ids: List[int],
    base_id: Optional[int] = None,
    validate: bool = True,
    tolerance: float = 1e-3,
) -> lanelet2.core.Lanelet:
    """
    Merge multiple Lanelets by their IDs from a lanelet map.

    This is a convenience function that retrieves lanelets by ID from a map
    and then merges them using merge_lanelets.

    Args:
        lanelet_map: The lanelet map containing the lanelets
        lanelet_ids: List of lanelet IDs to merge (must be in sequential order)
        base_id: Optional base ID for creating new IDs. If None, uses timestamp-based IDs.
        validate: If True, validates continuity between consecutive lanelets before merging
        tolerance: Distance tolerance for boundary continuity validation

    Returns:
        Single merged Lanelet object combining all specified lanelets

    Raises:
        ValueError: If any ID is not found in the map or if validation fails
    """
    # Retrieve lanelets from map by ID
    lanelets = []
    for lid in lanelet_ids:
        try:
            lanelet = lanelet_map.laneletLayer.get(lid)
            lanelets.append(lanelet)
        except RuntimeError:
            # lanelet2 throws RuntimeError when element not found
            raise ValueError(f"Lanelet with ID {lid} not found in map")

    # Merge the retrieved lanelets
    return merge_lanelets(lanelets, base_id, validate, tolerance)


def copy_map_excluding(
    lanelet_map: lanelet2.core.LaneletMap,
    exclude_ids: Set[int],
) -> lanelet2.core.LaneletMap:
    """Create a new LaneletMap with specified lanelets excluded.

    This is a shared helper that implements the common pattern of:
    1. Create a new LaneletMap
    2. Copy all lanelets except those in exclude_ids
    3. Copy all regulatory elements

    Args:
        lanelet_map: The source lanelet map
        exclude_ids: Set of lanelet IDs to exclude from the new map

    Returns:
        New LaneletMap without the excluded lanelets
    """
    new_map = lanelet2.core.LaneletMap()
    for ll in lanelet_map.laneletLayer:
        if ll.id not in exclude_ids:
            new_map.add(ll)
    for reg_elem in lanelet_map.regulatoryElementLayer:
        new_map.add(reg_elem)
    return new_map


def remove_lanelet(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelet: Union[lanelet2.core.Lanelet, int],
) -> lanelet2.core.LaneletMap:
    """
    Remove a specified lanelet from the lanelet map by creating a new map without it.

    Note: Since the Python bindings don't expose a direct remove method,
    this function creates a new map with all lanelets except the specified one.

    Args:
        lanelet_map: The lanelet map to remove from
        lanelet: Either a Lanelet object or lanelet ID to remove

    Returns:
        New LaneletMap with the specified lanelet removed
    """
    return remove_lanelets(lanelet_map, [lanelet])


def remove_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelets: List[Union[lanelet2.core.Lanelet, int]],
) -> lanelet2.core.LaneletMap:
    """
    Remove multiple lanelets from the lanelet map by creating a new map without them.

    Args:
        lanelet_map: The lanelet map to remove from
        lanelets: List of Lanelet objects or lanelet IDs to remove

    Returns:
        New LaneletMap with the specified lanelets removed
    """
    lanelet_ids_to_remove = {ll if isinstance(ll, int) else ll.id for ll in lanelets}
    return copy_map_excluding(lanelet_map, lanelet_ids_to_remove)


def replace_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelets: List[Union[lanelet2.core.Lanelet, int]],
    validate: bool = True,
    tolerance: float = 1e-3,
) -> lanelet2.core.LaneletMap:
    """
    Replace multiple lanelets in the map with a single merged lanelet.

    This function merges multiple lanelets into one using merge_lanelets,
    assigns a new ID (max lanelet ID in map + 1), removes the original lanelets,
    and adds the new merged lanelet to the map.

    Args:
        lanelet_map: The lanelet map to operate on
        lanelets: List of lanelets or lanelet IDs to merge and replace
        validate: If True, validates continuity between consecutive lanelets before merging
        tolerance: Distance tolerance for boundary continuity validation

    Returns:
        New LaneletMap with the lanelets replaced by the merged lanelet

    Raises:
        ValueError: If less than 2 lanelets provided, if any lanelet is not found,
                   or if validation fails
    """
    if not lanelets or len(lanelets) < 2:
        raise ValueError("At least 2 lanelets required for replacement")

    # Convert input to actual lanelet objects
    lanelet_objects = []
    lanelet_ids_to_remove = set()

    for lanelet in lanelets:
        if isinstance(lanelet, int):
            # It's an ID, need to get the lanelet from the map
            try:
                ll = lanelet_map.laneletLayer.get(lanelet)
                lanelet_objects.append(ll)
                lanelet_ids_to_remove.add(lanelet)
            except RuntimeError:
                raise ValueError(f"Lanelet with ID {lanelet} not found in map")
        else:
            # It's already a lanelet object
            # Verify it exists in the map
            try:
                lanelet_map.laneletLayer.get(lanelet.id)
                lanelet_objects.append(lanelet)
                lanelet_ids_to_remove.add(lanelet.id)
            except RuntimeError:
                raise ValueError(f"Lanelet with ID {lanelet.id} not found in map")

    # Find the maximum lanelet ID in the map
    max_id = 0
    for ll in lanelet_map.laneletLayer:
        if ll.id > max_id:
            max_id = ll.id

    # Generate new base ID for the merged lanelet (max ID + 1)
    # We need IDs for: left boundary, right boundary, and the lanelet itself
    # So we use max_id + 1, max_id + 2, max_id + 3
    new_base_id = max_id

    # Merge the lanelets using merge_lanelets
    merged_lanelet = merge_lanelets(
        lanelet_objects, base_id=new_base_id, validate=validate, tolerance=tolerance
    )

    # Create a new map with the replacement, then add the merged lanelet
    new_map = copy_map_excluding(lanelet_map, lanelet_ids_to_remove)
    new_map.add(merged_lanelet)

    return new_map


def get_max_lanelet_id(lanelet_map: lanelet2.core.LaneletMap) -> int:
    """
    Get the maximum lanelet ID in the map.

    Args:
        lanelet_map: The lanelet map to search

    Returns:
        Maximum lanelet ID in the map, or 0 if map is empty
    """
    max_id = 0
    for ll in lanelet_map.laneletLayer:
        if ll.id > max_id:
            max_id = ll.id
    return max_id


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
