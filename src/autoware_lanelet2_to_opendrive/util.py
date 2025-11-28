"""Utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import Set, List
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
                        If not empty, includes target lanelets and their left/right adjacent lanelets.

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

        # Add left/right adjacent lanelets to the grouping set
        for target_ll in target_lanelets:
            # Add left adjacent lanelets
            left_ll = temp_routing_graph.left(target_ll)
            if left_ll:
                lanelets_to_group.add(left_ll)

            # Add right adjacent lanelets
            right_ll = temp_routing_graph.right(target_ll)
            if right_ll:
                lanelets_to_group.add(right_ll)

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
