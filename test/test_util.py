"""Tests for utility functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.util import (
    find_lanelets_without_next,
    find_lanelets_without_previous,
    find_terminal_lanelets,
)


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_find_lanelets_without_next():
    """Test finding lanelets without successors."""
    lanelet_map = load_test_map()

    # Find lanelets without next
    terminal_lanelets = find_lanelets_without_next(lanelet_map)

    # Check that we found some terminal lanelets
    assert isinstance(terminal_lanelets, set)
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in terminal_lanelets)

    # Check that lanelet 3002082 is included in terminal lanelets
    terminal_ids = {ll.id for ll in terminal_lanelets}
    assert (
        3002082 in terminal_ids
    ), f"Lanelet 3002082 should be in terminal lanelets, but found: {terminal_ids}"

    # Verify these lanelets actually have no following lanelets
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    for lanelet in terminal_lanelets:
        following = routing_graph.following(lanelet)
        assert (
            len(following) == 0
        ), f"Lanelet {lanelet.id} has following lanelets but was marked as terminal"


def test_find_lanelets_without_previous():
    """Test finding lanelets without predecessors."""
    lanelet_map = load_test_map()

    # Find lanelets without previous
    start_lanelets = find_lanelets_without_previous(lanelet_map)

    # Check that we found some start lanelets
    assert isinstance(start_lanelets, set)
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in start_lanelets)

    # Check that lanelet 3002084 is included in start lanelets
    start_ids = {ll.id for ll in start_lanelets}
    assert (
        3002084 in start_ids
    ), f"Lanelet 3002084 should be in start lanelets, but found: {start_ids}"

    # Verify these lanelets actually have no previous lanelets
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    for lanelet in start_lanelets:
        previous = routing_graph.previous(lanelet)
        assert (
            len(previous) == 0
        ), f"Lanelet {lanelet.id} has previous lanelets but was marked as start"


def test_find_terminal_lanelets():
    """Test finding both start and end terminal lanelets."""
    lanelet_map = load_test_map()

    # Find terminal lanelets
    start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)

    # Check that both sets are returned
    assert isinstance(start_lanelets, set)
    assert isinstance(end_lanelets, set)

    # Check that all elements are lanelets
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in start_lanelets)
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in end_lanelets)

    # Verify consistency with individual functions
    start_lanelets_individual = find_lanelets_without_previous(lanelet_map)
    end_lanelets_individual = find_lanelets_without_next(lanelet_map)

    assert start_lanelets == start_lanelets_individual
    assert end_lanelets == end_lanelets_individual


def test_terminal_lanelets_completeness():
    """Test that all lanelets are properly categorized."""
    lanelet_map = load_test_map()

    # Get terminal lanelets
    start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)

    # Create routing graph for verification
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    # Check all lanelets and verify terminal classification
    for lanelet in lanelet_map.laneletLayer:
        has_previous = len(routing_graph.previous(lanelet)) > 0
        has_following = len(routing_graph.following(lanelet)) > 0

        # Verify start lanelet classification
        if not has_previous:
            assert (
                lanelet in start_lanelets
            ), f"Lanelet {lanelet.id} has no previous but not in start_lanelets"
        else:
            assert (
                lanelet not in start_lanelets
            ), f"Lanelet {lanelet.id} has previous but in start_lanelets"

        # Verify end lanelet classification
        if not has_following:
            assert (
                lanelet in end_lanelets
            ), f"Lanelet {lanelet.id} has no following but not in end_lanelets"
        else:
            assert (
                lanelet not in end_lanelets
            ), f"Lanelet {lanelet.id} has following but in end_lanelets"


def test_empty_map():
    """Test functions with an empty map."""
    # Create empty map
    empty_map = lanelet2.core.LaneletMap()

    # Test all functions with empty map
    start_lanelets = find_lanelets_without_previous(empty_map)
    end_lanelets = find_lanelets_without_next(empty_map)
    terminal_start, terminal_end = find_terminal_lanelets(empty_map)

    # All should return empty sets
    assert len(start_lanelets) == 0
    assert len(end_lanelets) == 0
    assert len(terminal_start) == 0
    assert len(terminal_end) == 0
