"""Utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import Set, List, Union, Dict, Optional
from enum import Enum
from dataclasses import dataclass
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from lanelet2.geometry import intersects2d
import mgrs


@dataclass
class RoadLaneletMapping:
    """
    Mapping between OpenDRIVE Roads and Lanelet2 lanelets.

    This class provides bidirectional mapping to easily convert between
    OpenDRIVE road IDs and Lanelet2 lanelet IDs.

    Attributes:
        road_to_lanelets: Maps OpenDRIVE road ID to list of Lanelet2 lanelet IDs
        lanelet_to_road: Maps Lanelet2 lanelet ID to OpenDRIVE road ID
    """

    road_to_lanelets: Dict[int, List[int]]
    lanelet_to_road: Dict[int, int]

    def get_lanelets_for_road(self, road_id: int) -> List[int]:
        """Get all lanelet IDs that belong to a specific road.

        Args:
            road_id: OpenDRIVE road ID

        Returns:
            List of Lanelet2 lanelet IDs, or empty list if road not found
        """
        return self.road_to_lanelets.get(road_id, [])

    def get_road_for_lanelet(self, lanelet_id: int) -> Optional[int]:
        """Get the road ID that contains a specific lanelet.

        Args:
            lanelet_id: Lanelet2 lanelet ID

        Returns:
            OpenDRIVE road ID, or None if lanelet not found
        """
        return self.lanelet_to_road.get(lanelet_id)


class ConnectionDirection(Enum):
    """Enum for specifying connection direction in lanelet relationships."""

    FOLLOWING = "following"
    PREVIOUS = "previous"


def find_lanelets_without_next(
    lanelet_map: lanelet2.core.LaneletMap,
) -> Set[lanelet2.core.Lanelet]:
    """Find all lanelets that have no following lanelets.

    Args:
        lanelet_map: The lanelet2 map to analyze

    Returns:
        Set of lanelets that have no successors
    """
    # Create routing graph for the map
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])

    lanelets_without_next = set()

    for lanelet in lanelet_map.laneletLayer:
        # Check if this lanelet has any following lanelets
        following = routing_graph.following(lanelet)
        if not following:
            lanelets_without_next.add(lanelet)

    return lanelets_without_next


def find_lanelets_without_previous(
    lanelet_map: lanelet2.core.LaneletMap,
) -> Set[lanelet2.core.Lanelet]:
    """Find all lanelets that have no preceding lanelets.

    Args:
        lanelet_map: The lanelet2 map to analyze

    Returns:
        Set of lanelets that have no predecessors
    """
    # Create routing graph for the map
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])

    lanelets_without_previous = set()

    for lanelet in lanelet_map.laneletLayer:
        # Check if this lanelet has any preceding lanelets
        previous = routing_graph.previous(lanelet)
        if not previous:
            lanelets_without_previous.add(lanelet)

    return lanelets_without_previous


def find_terminal_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
) -> tuple[Set[lanelet2.core.Lanelet], Set[lanelet2.core.Lanelet]]:
    """Find all terminal lanelets (start and end points) in the map.

    Args:
        lanelet_map: The lanelet2 map to analyze

    Returns:
        Tuple of (lanelets_without_previous, lanelets_without_next)
    """
    # Create routing graph once for efficiency
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])

    lanelets_without_previous = set()
    lanelets_without_next = set()

    for lanelet in lanelet_map.laneletLayer:
        # Check for previous lanelets
        if not routing_graph.previous(lanelet):
            lanelets_without_previous.add(lanelet)

        # Check for following lanelets
        if not routing_graph.following(lanelet):
            lanelets_without_next.add(lanelet)

    return lanelets_without_previous, lanelets_without_next


def find_connecting_lanelet_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelet_group: Union[
        Set[lanelet2.core.Lanelet],
        List[lanelet2.core.Lanelet],
        lanelet2.core.LaneletLayer,
    ],
    direction: ConnectionDirection,
    routing_graph: Optional[RoutingGraph] = None,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Find and group the connecting lanelets of a given lanelet group.

    Args:
        lanelet_map: The lanelet2 map to analyze
        lanelet_group: Set of lanelets to find connections for
        direction: ConnectionDirection enum value to specify which connections to find
        routing_graph: Optional pre-built routing graph. If None, creates a new one.

    Returns:
        List of sets, where each set contains lanelets that are adjacent to each other
    """
    # Use provided routing graph or create a new one
    if routing_graph is None:
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

    # Collect all connecting lanelets
    connecting_lanelets = set()
    for lanelet in lanelet_group:
        if direction == ConnectionDirection.FOLLOWING:
            connections = routing_graph.following(lanelet)
        elif direction == ConnectionDirection.PREVIOUS:
            connections = routing_graph.previous(lanelet)
        else:
            # This should never happen with enum, but provides safety
            raise ValueError(f"Invalid direction: {direction}")

        for connected_ll in connections:
            connecting_lanelets.add(connected_ll)

    # Group the connecting lanelets by their adjacency
    groups = find_adjacent_groups(lanelet_map, connecting_lanelets, routing_graph)

    return groups


def find_adjacent_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    target_lanelets: Set[lanelet2.core.Lanelet],
    routing_graph: Optional[RoutingGraph] = None,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Find groups of laterally adjacent lanelets.

    Groups lanelets that are connected to each other by left/right adjacency
    into separate groups. Does not consider longitudinal (following/previous) connections.

    Args:
        lanelet_map: The lanelet2 map to analyze
        target_lanelets: Set of lanelets to group. If empty, groups all lanelets in the map.
                        If not empty, groups only the target lanelets into adjacent groups.
        routing_graph: Optional pre-built routing graph. If None, creates a new one.

    Returns:
        List of sets, where each set contains lanelets that are laterally adjacent to each other
    """
    all_lanelets_in_map = set(lanelet_map.laneletLayer)

    # If target_lanelets is empty, use all lanelets from the map
    if not target_lanelets:
        lanelets_to_group = all_lanelets_in_map
    else:
        # Use only the target lanelets without adding adjacent ones from other groups
        # The adjacency relationships will be found in the DFS below
        lanelets_to_group = target_lanelets.copy()

    # Use provided routing graph or create a new one
    if routing_graph is None:
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

    groups = []
    visited = set()

    def get_adjacent_lanelets(lanelet):
        """Get laterally adjacent lanelets (left/right only, not following/previous)."""
        adjacent = set()

        # Add left adjacent lanelets
        left_ll = routing_graph.left(lanelet)
        if left_ll and left_ll in lanelets_to_group:
            adjacent.add(left_ll)

        # Add right adjacent lanelets
        right_ll = routing_graph.right(lanelet)
        if right_ll and right_ll in lanelets_to_group:
            adjacent.add(right_ll)

        return adjacent

    def dfs_group(start_lanelet, current_group):
        """Depth-first search to find all connected lanelets."""
        if start_lanelet in visited:
            return

        visited.add(start_lanelet)
        current_group.add(start_lanelet)

        # Recursively add all adjacent lanelets
        for adjacent_ll in get_adjacent_lanelets(start_lanelet):
            if adjacent_ll not in visited:
                dfs_group(adjacent_ll, current_group)

    # Group lanelets using DFS
    for lanelet in lanelets_to_group:
        if lanelet not in visited:
            current_group: Set[lanelet2.core.Lanelet] = set()
            dfs_group(lanelet, current_group)
            if current_group:
                groups.append(current_group)

    return groups


def filter_lanelets_by_subtype(
    lanelets: Union[
        Set[lanelet2.core.Lanelet],
        List[lanelet2.core.Lanelet],
        lanelet2.core.LaneletLayer,
    ],
    subtypes: List[str],
) -> Set[lanelet2.core.Lanelet]:
    """Filter lanelets by their subtype attribute.

    Args:
        lanelets: Collection of lanelets to filter. Can be a set, list, or LaneletLayer.
        subtypes: List of subtypes to filter by. Lanelets with any of these
                  subtypes will be returned.

    Returns:
        Set of lanelets that match the specified subtype(s).
        If subtypes is empty, returns empty set.
        If a lanelet doesn't have a subtype attribute, it is not included.

    Examples:
        # Filter for road lanelets only
        road_lanelets = filter_lanelets_by_subtype(lanelet_map.laneletLayer, ["road"])

        # Filter for multiple subtypes
        main_lanelets = filter_lanelets_by_subtype(
            lanelet_map.laneletLayer,
            ["road", "highway"]
        )

        # Filter from a specific set of lanelets
        terminal_roads = filter_lanelets_by_subtype(terminal_lanelets, ["road"])
    """
    if not subtypes:
        return set()

    # Convert input to a set for consistent processing
    if isinstance(lanelets, set):
        lanelet_set = lanelets
    elif isinstance(lanelets, list):
        lanelet_set = set(lanelets)
    elif isinstance(lanelets, lanelet2.core.LaneletLayer):
        lanelet_set = set(lanelets)
    else:
        # Try to convert any iterable to set
        try:
            lanelet_set = set(lanelets)
        except TypeError:
            raise TypeError(
                f"lanelets must be a set, list, or LaneletLayer, got {type(lanelets)}"
            )

    # Prepare the set of subtypes to check
    target_subtypes = set(subtypes)

    # Filter lanelets by subtype
    filtered_lanelets = set()
    for lanelet in lanelet_set:
        # Check if the lanelet has a subtype attribute
        if lanelet.attributes and "subtype" in lanelet.attributes:
            lanelet_subtype = lanelet.attributes["subtype"]
            if lanelet_subtype in target_subtypes:
                filtered_lanelets.add(lanelet)

    return filtered_lanelets


def check_lanelet_groups_intersect(
    group1: Union[
        Set[lanelet2.core.Lanelet],
        List[lanelet2.core.Lanelet],
        lanelet2.core.LaneletLayer,
    ],
    group2: Union[
        Set[lanelet2.core.Lanelet],
        List[lanelet2.core.Lanelet],
        lanelet2.core.LaneletLayer,
    ],
) -> bool:
    """Check if two lanelet groups have any intersecting lanelets.

    Args:
        group1: First group of lanelets
        group2: Second group of lanelets

    Returns:
        True if any lanelet in group1 intersects with any lanelet in group2
    """
    for lanelet_1 in group1:
        for lanelet_2 in group2:
            if intersects2d(lanelet_1, lanelet_2):
                return True
    return False


def sort_adjacent_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    target_lanelets: Union[
        Set[lanelet2.core.Lanelet],
        List[lanelet2.core.Lanelet],
        lanelet2.core.LaneletLayer,
    ],
) -> List[lanelet2.core.Lanelet]:
    """Sort lanelets from left to right by following their adjacent relationships.

    Args:
        lanelet_map: The lanelet2 map containing the lanelets
        target_lanelets: Set of lanelets to sort

    Returns:
        List of lanelets sorted from left to right

    Raises:
        ValueError: If target_lanelets contains non-adjacent lanelets
    """
    if not target_lanelets:
        return []

    # Create routing graph for finding adjacent relationships
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])

    # Find the leftmost lanelet by traversing left until no more left neighbors
    def find_leftmost_lanelet(
        start_lanelet: lanelet2.core.Lanelet,
    ) -> lanelet2.core.Lanelet:
        current = start_lanelet
        while True:
            left_neighbor = routing_graph.left(current)
            if left_neighbor and left_neighbor in target_lanelets:
                current = left_neighbor
            else:
                break
        return current

    # Start with any lanelet from the set
    start_lanelet = next(iter(target_lanelets))
    leftmost = find_leftmost_lanelet(start_lanelet)

    # Build sorted list from left to right
    sorted_lanelets = []
    remaining_lanelets = target_lanelets.copy()
    current = leftmost

    while current and current in remaining_lanelets:
        sorted_lanelets.append(current)
        remaining_lanelets.remove(current)

        # Move to the right neighbor
        right_neighbor = routing_graph.right(current)
        if right_neighbor and right_neighbor in remaining_lanelets:
            current = right_neighbor
        else:
            break

    # Check if all lanelets were processed (i.e., all are adjacent)
    if remaining_lanelets:
        remaining_ids = [ll.id for ll in remaining_lanelets]
        raise ValueError(
            f"Target lanelets contain non-adjacent lanelets. "
            f"Non-adjacent lanelet IDs: {remaining_ids}"
        )

    return sorted_lanelets


def mgrs_to_lanelet2_origin(mgrs_grid: str) -> lanelet2.io.Origin:
    """Convert MGRS grid name to lanelet2.io.Origin.

    If the input is a partial MGRS grid (e.g., "54SUE" without meter coordinates),
    it will be zero-padded to get the origin coordinates of that grid.

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE1234567890" or "54SUE")

    Returns:
        lanelet2.io.Origin object with coordinates converted from MGRS

    Raises:
        ValueError: If the MGRS grid string is invalid
    """
    try:
        # Create MGRS converter
        m = mgrs.MGRS()

        # Handle partial MGRS grid by padding with zeros if needed
        processed_mgrs = mgrs_grid.strip()

        # Check if we have a partial grid (missing meter coordinates)
        # Full MGRS format: ZONE BAND SQUARE_ID EASTING NORTHING
        # e.g., "54SUE1234567890" where "54S" is zone, "UE" is square, "1234567890" is coordinates
        if len(processed_mgrs) >= 3:
            # Extract the grid zone designator and square identifier
            # Typical format: [0-9]+[A-Z][A-Z][A-Z]
            import re

            match = re.match(r"^(\d+[A-Z][A-Z][A-Z])(.*)$", processed_mgrs)
            if match:
                grid_square = match.group(1)
                coordinates = match.group(2)

                # If coordinates are missing or incomplete, pad with zeros
                if len(coordinates) == 0:
                    # No coordinates provided, use origin (00000 00000)
                    processed_mgrs = grid_square + "0000000000"
                elif len(coordinates) < 10:
                    # Partial coordinates provided
                    # MGRS coordinates should be even length (easting + northing pairs)
                    # If odd length, pad to even, then pad to 10 total
                    if len(coordinates) % 2 == 1:
                        # Odd length - pad one zero to make even pairs
                        coordinates += "0"

                    # Now pad to 10 digits total (5 easting + 5 northing)
                    padded_coords = coordinates.ljust(10, "0")
                    processed_mgrs = grid_square + padded_coords

        # Convert MGRS to latitude/longitude
        lat, lon = m.toLatLon(processed_mgrs)

        # Create lanelet2 Origin with the converted coordinates
        origin = lanelet2.io.Origin(lat, lon)

        return origin

    except Exception as e:
        raise ValueError(f"Invalid MGRS grid string '{mgrs_grid}': {e}") from e


def mgrs_to_proj_string(mgrs_grid: str) -> str:
    """Convert MGRS grid to PROJ string for OpenDRIVE geoReference.

    Args:
        mgrs_grid: MGRS grid reference string (e.g., "54SUE" or "54SUE1234567890")

    Returns:
        PROJ string with UTM projection and origin coordinates from MGRS grid

    Raises:
        ValueError: If the MGRS grid string is invalid

    Example:
        >>> mgrs_to_proj_string("54SUE")
        '+proj=utm +zone=54 +south +lat_0=-28.0 +lon_0=141.0 +datum=WGS84 +units=m +no_defs'
    """
    import re

    try:
        # Extract UTM zone number and latitude band
        match = re.match(r"^(\d+)([A-Z])", mgrs_grid)
        if not match:
            raise ValueError(f"Invalid MGRS format: {mgrs_grid}")

        zone = match.group(1)
        band = match.group(2)

        # Determine hemisphere from latitude band
        # Latitude bands: C-M are south, N-X are north
        is_south = band < "N"
        hemisphere = "+south" if is_south else ""

        # Get origin lat/lon from MGRS grid using mgrs library directly
        m = mgrs.MGRS()

        # Handle partial MGRS grid by padding with zeros if needed
        processed_mgrs = mgrs_grid.strip()
        match_full = re.match(r"^(\d+[A-Z][A-Z][A-Z])(.*)$", processed_mgrs)
        if match_full:
            grid_square = match_full.group(1)
            coordinates = match_full.group(2)

            # If coordinates are missing, use origin (00000 00000)
            if len(coordinates) == 0:
                processed_mgrs = grid_square + "0000000000"
            elif len(coordinates) < 10:
                # Partial coordinates - pad to 10 digits
                if len(coordinates) % 2 == 1:
                    coordinates += "0"
                padded_coords = coordinates.ljust(10, "0")
                processed_mgrs = grid_square + padded_coords

        # Convert MGRS to latitude/longitude
        lat, lon = m.toLatLon(processed_mgrs)

        # Build PROJ string with UTM projection and MGRS origin coordinates
        proj_string = (
            f"+proj=utm +zone={zone} {hemisphere} "
            f"+lat_0={lat} +lon_0={lon} "
            f"+datum=WGS84 +units=m +no_defs"
        ).replace("  ", " ")  # Remove double spaces if hemisphere is empty

        return proj_string

    except Exception as e:
        raise ValueError(f"Invalid MGRS grid string '{mgrs_grid}': {e}") from e


def filter_regulatory_element_by_type(
    lanelet_map: lanelet2.core.LaneletMap,
    element_type: str,
) -> Dict[int, tuple[lanelet2.core.RegulatoryElement, Set[int]]]:
    """
    Filter regulatory elements of specified type from lanelet map.

    Args:
        lanelet_map: Lanelet2 map containing regulatory elements
        element_type: Type of regulatory element to filter (e.g., "trafficLights", "speedLimits")

    Returns:
        Dictionary mapping regulatory element ID to (regulatory element, set of lanelet IDs)
    """
    element_map: Dict[int, tuple[lanelet2.core.RegulatoryElement, Set[int]]] = {}

    for lanelet in lanelet_map.laneletLayer:
        # Get all regulatory elements of the specified type for this lanelet
        for reg_elem in lanelet.regulatoryElements:
            # Check if this regulatory element has the specified type attribute
            if hasattr(reg_elem, element_type) and getattr(reg_elem, element_type):
                reg_elem_id = reg_elem.id
                if reg_elem_id not in element_map:
                    element_map[reg_elem_id] = (reg_elem, set())
                element_map[reg_elem_id][1].add(lanelet.id)

    return element_map
