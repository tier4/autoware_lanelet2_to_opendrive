"""Functions for working with Lanelet2 lanelet objects."""

from typing import Optional, List, Union
import lanelet2
import numpy as np


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
    # Handle both Lanelet object and ID input
    if isinstance(lanelet, int):
        lanelet_id = lanelet
    else:
        lanelet_id = lanelet.id

    # Create a new map
    new_map = lanelet2.core.LaneletMap()

    # Copy all lanelets except the one to remove
    for ll in lanelet_map.laneletLayer:
        if ll.id != lanelet_id:
            new_map.add(ll)

    # Copy regulatory elements
    for reg_elem in lanelet_map.regulatoryElementLayer:
        new_map.add(reg_elem)

    return new_map


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
    # Convert all inputs to IDs
    lanelet_ids_to_remove = set()
    for lanelet in lanelets:
        if isinstance(lanelet, int):
            lanelet_ids_to_remove.add(lanelet)
        else:
            lanelet_ids_to_remove.add(lanelet.id)

    # Create a new map
    new_map = lanelet2.core.LaneletMap()

    # Copy all lanelets except those to remove
    for ll in lanelet_map.laneletLayer:
        if ll.id not in lanelet_ids_to_remove:
            new_map.add(ll)

    # Copy regulatory elements
    for reg_elem in lanelet_map.regulatoryElementLayer:
        new_map.add(reg_elem)

    return new_map
