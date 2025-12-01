"""Utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import Set, List, Optional, Union
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance


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


def find_adjacent_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    target_lanelets: Set[lanelet2.core.Lanelet],
) -> List[Set[lanelet2.core.Lanelet]]:
    """Find groups of adjacent lanelets.

    Groups lanelets that are connected to each other (either by following/previous
    or left/right adjacency) into separate groups.

    Args:
        lanelet_map: The lanelet2 map to analyze
        target_lanelets: Set of lanelets to group. If empty, groups all lanelets in the map.
                        If not empty, includes target lanelets and recursively all their left/right adjacent lanelets.

    Returns:
        List of sets, where each set contains lanelets that are adjacent to each other
    """
    all_lanelets_in_map = set(lanelet_map.laneletLayer)

    # If target_lanelets is empty, use all lanelets from the map
    if not target_lanelets:
        lanelets_to_group = all_lanelets_in_map
    else:
        # Start with target lanelets and add their adjacent lanelets
        lanelets_to_group = target_lanelets.copy()

        # Create routing graph to find adjacent lanelets
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        temp_routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Recursively add all left/right adjacent lanelets to the grouping set
        visited_for_expansion = set()

        def add_left_right_recursively(lanelet):
            """Recursively add all left and right adjacent lanelets."""
            if lanelet in visited_for_expansion:
                return

            visited_for_expansion.add(lanelet)
            lanelets_to_group.add(lanelet)

            # Add left adjacent lanelets recursively
            left_ll = temp_routing_graph.left(lanelet)
            if left_ll and left_ll not in visited_for_expansion:
                add_left_right_recursively(left_ll)

            # Add right adjacent lanelets recursively
            right_ll = temp_routing_graph.right(lanelet)
            if right_ll and right_ll not in visited_for_expansion:
                add_left_right_recursively(right_ll)

        # Start recursion from all target lanelets
        for target_ll in target_lanelets:
            add_left_right_recursively(target_ll)

    # Create routing graph for connectivity analysis
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])

    groups = []
    visited = set()

    def get_adjacent_lanelets(lanelet):
        """Get all lanelets adjacent to the given lanelet."""
        adjacent = set()

        # Add following lanelets
        for following_ll in routing_graph.following(lanelet):
            if following_ll in lanelets_to_group:
                adjacent.add(following_ll)

        # Add previous lanelets
        for previous_ll in routing_graph.previous(lanelet):
            if previous_ll in lanelets_to_group:
                adjacent.add(previous_ll)

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
    subtype: Optional[str] = None,
    subtypes: Optional[List[str]] = None,
) -> Set[lanelet2.core.Lanelet]:
    """Filter lanelets by their subtype attribute.

    Args:
        lanelets: Collection of lanelets to filter. Can be a set, list, or LaneletLayer.
        subtype: Single subtype to filter by (e.g., "road", "highway", "merging").
                 If specified, only lanelets with this exact subtype will be returned.
        subtypes: List of subtypes to filter by. If specified, lanelets with any of these
                  subtypes will be returned. Cannot be used together with 'subtype'.

    Returns:
        Set of lanelets that match the specified subtype(s).
        If neither subtype nor subtypes is specified, returns empty set.
        If a lanelet doesn't have a subtype attribute, it is not included.

    Raises:
        ValueError: If both 'subtype' and 'subtypes' are specified.

    Examples:
        # Filter for road lanelets only
        road_lanelets = filter_lanelets_by_subtype(lanelet_map.laneletLayer, subtype="road")

        # Filter for multiple subtypes
        main_lanelets = filter_lanelets_by_subtype(
            lanelet_map.laneletLayer,
            subtypes=["road", "highway"]
        )

        # Filter from a specific set of lanelets
        terminal_roads = filter_lanelets_by_subtype(terminal_lanelets, subtype="road")
    """
    if subtype is not None and subtypes is not None:
        raise ValueError("Cannot specify both 'subtype' and 'subtypes' parameters")

    if subtype is None and subtypes is None:
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

    # Prepare the list of subtypes to check
    if subtype is not None:
        target_subtypes = {subtype}
    else:
        # subtypes is guaranteed to not be None here due to earlier check
        target_subtypes = set(subtypes) if subtypes is not None else set()

    # Filter lanelets by subtype
    filtered_lanelets = set()
    for lanelet in lanelet_set:
        # Check if the lanelet has a subtype attribute
        if lanelet.attributes and "subtype" in lanelet.attributes:
            lanelet_subtype = lanelet.attributes["subtype"]
            if lanelet_subtype in target_subtypes:
                filtered_lanelets.add(lanelet)

    return filtered_lanelets
