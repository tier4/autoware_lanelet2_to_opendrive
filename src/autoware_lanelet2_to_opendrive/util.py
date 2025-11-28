"""Utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import Set
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
